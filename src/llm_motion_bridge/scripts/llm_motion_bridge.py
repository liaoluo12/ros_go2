#!/usr/bin/env python3
"""ROS node that delegates velocity planning to an external LLM API."""
import json
import os
import threading
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urljoin

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

try:
    import requests
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise ImportError("The llm_motion_bridge node requires the 'requests' package. Install it with 'sudo apt install python3-requests'.") from exc


DEFAULT_SYSTEM_PROMPT = (
    "You control a mobile robot. Given an instruction, reply with JSON using the schema "
    '{"linear": {"x": <float>, "y": <float>, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": <float>}, '
    '"comment": "<optional short remark>"}. Values represent velocities in m/s and rad/s. '
    "Stay within the requested speed limits and do not return any non-JSON text."
)


class LLMMotionBridge:
    """Translate natural-language requests into velocity commands via LLM."""

    def __init__(self) -> None:
        self.api_base = rospy.get_param("~api_base", "")
        self.api_path = rospy.get_param("~api_path", "/chat/completions")
        self.api_url = rospy.get_param("~api_url", "")  # backwards compatibility

        if self.api_url:
            rospy.logwarn_once("Parameter '~api_url' is deprecated; prefer '~api_base' and '~api_path'")
            self.api_endpoint = self.api_url
        else:
            if not self.api_base:
                rospy.logfatal("Parameter '~api_base' is required for llm_motion_bridge")
                raise ValueError("api_base parameter not set")
            self.api_endpoint = urljoin(self.api_base.rstrip('/') + '/', self.api_path.lstrip('/'))

        self.model = rospy.get_param("~model", "gpt-4.1-mini")
        self.temperature = float(rospy.get_param("~temperature", 0.1))
        self.system_prompt = rospy.get_param("~system_prompt", DEFAULT_SYSTEM_PROMPT)
        self.response_format = rospy.get_param("~response_format", "json_object")
        self.api_timeout = float(rospy.get_param("~api_timeout", 20.0))

        self.api_key = rospy.get_param("~api_key", "")
        if not self.api_key:
            api_key_env = rospy.get_param("~api_key_env", "LLM_API_KEY")
            self.api_key = os.environ.get(api_key_env, "")
        if not self.api_key:
            rospy.logwarn("No API key provided. Requests may fail if the endpoint requires authentication.")

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.instruction_topic = rospy.get_param("~instruction_topic", "/llm_motion/instruction")

        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.command_hold_duration = float(rospy.get_param("~command_hold_duration", 1.5))

        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.6))
        self.max_side_speed = float(rospy.get_param("~max_side_speed", 0.2))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.2))
        self.allow_y_motion = bool(rospy.get_param("~allow_y_motion", False))

        self.session = requests.Session()
        self.session.headers.update({"Content-Type": "application/json"})
        if self.api_key:
            auth_header = rospy.get_param("~api_key_header", "Authorization")
            self.session.headers[auth_header] = f"Bearer {self.api_key}"

        self.cmd_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self._command_lock = threading.Lock()
        self._current_cmd = Twist()
        self._last_command_time = rospy.Time(0.0)

        rospy.Subscriber(self.instruction_topic, String, self._instruction_callback, queue_size=5)
        self._publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._timer_publish)
        self._stop_service = rospy.Service("~stop", Trigger, self._handle_stop)

        rospy.loginfo("llm_motion_bridge ready. Listening on %s", self.instruction_topic)

    def _instruction_callback(self, msg: String) -> None:
        instruction = msg.data.strip()
        if not instruction:
            rospy.logdebug("Received empty instruction; ignoring")
            return

        rospy.loginfo("Forwarding instruction to LLM: %s", instruction)
        try:
            payload = self._build_payload(instruction)
            response = self.session.post(self.api_endpoint, json=payload, timeout=self.api_timeout)
            response.raise_for_status()
            twist_data = self._extract_twist(response.json())
            twist_cmd, comment = self._to_twist(twist_data)
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("LLM request failed: %s", exc)
            self._apply_stop()
            return

        with self._command_lock:
            self._current_cmd = twist_cmd
            self._last_command_time = rospy.Time.now()

        if comment:
            rospy.loginfo("LLM comment: %s", comment)

    def _build_payload(self, instruction: str) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": instruction},
            ],
            "temperature": self.temperature,
        }
        if self.response_format:
            payload["response_format"] = {"type": self.response_format}
        return payload

    def _extract_twist(self, response_json: Dict[str, Any]) -> Dict[str, Any]:
        """Pull JSON payload from an OpenAI-compatible response."""
        if "choices" in response_json:
            choice = response_json["choices"][0]
            message = choice.get("message", {})
            content = message.get("content", "") if isinstance(message, dict) else message
        else:
            content = response_json.get("content", "")

        if isinstance(content, dict):
            return content

        text = str(content).strip()
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end == -1:
            raise ValueError("LLM response does not contain JSON content")
        try:
            return json.loads(text[start : end + 1])
        except json.JSONDecodeError as exc:
            raise ValueError(f"Failed to parse JSON from LLM response: {text}") from exc

    def _to_twist(self, data: Dict[str, Any]) -> Tuple[Twist, Optional[str]]:
        twist = Twist()
        linear = data.get("linear", {})
        angular = data.get("angular", {})

        twist.linear.x = self._clamp(float(linear.get("x", 0.0)), self.max_linear_speed)
        y_target = float(linear.get("y", 0.0))
        twist.linear.y = self._clamp(y_target, self.max_side_speed) if self.allow_y_motion else 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self._clamp(float(angular.get("z", 0.0)), self.max_angular_speed)

        comment = data.get("comment")
        if isinstance(comment, str):
            comment = comment.strip()
        else:
            comment = None
        return twist, comment

    def _clamp(self, value: float, limit: float) -> float:
        if limit <= 0.0:
            return 0.0
        return max(min(value, limit), -limit)

    def _timer_publish(self, _: Any) -> None:
        with self._command_lock:
            age = (rospy.Time.now() - self._last_command_time).to_sec()
            if age > self.command_hold_duration:
                publish_cmd = Twist()
            else:
                publish_cmd = self._current_cmd
        self.cmd_publisher.publish(publish_cmd)

    def _apply_stop(self) -> None:
        with self._command_lock:
            self._current_cmd = Twist()
            self._last_command_time = rospy.Time(0.0)
        self.cmd_publisher.publish(Twist())

    def _handle_stop(self, _: Any) -> TriggerResponse:
        self._apply_stop()
        return TriggerResponse(success=True, message="Command reset to zero velocities")


def main() -> None:
    rospy.init_node("llm_motion_bridge")
    bridge = LLMMotionBridge()
    rospy.spin()
    bridge._apply_stop()  # ensure robot stops on node shutdown


if __name__ == "__main__":
    main()
