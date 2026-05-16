"""
Audio alert node — run on the ground station (not the drone).

Exposes a ROS2 service /play_audio (std_srvs/SetBool) called by mission_3
when a pressure measurement is ready:
  request.data = True  → pressure above limit → plays audio_above_limit
  request.data = False → pressure within limit → plays audio_below_limit

Only one audio plays at a time: if the previous process is still running
the request is accepted but the new audio is skipped (success=False).

Requires 'mpg123':  sudo apt install mpg123
Fallback TTS:       sudo apt install espeak-ng
"""

import subprocess
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class AudioAlertNode(Node):
    def __init__(self):
        super().__init__('audio_alert')

        self.declare_parameter('audio_above_limit', '')
        self.declare_parameter('audio_below_limit', '')

        self._audio_above = self.get_parameter('audio_above_limit').value
        self._audio_below = self.get_parameter('audio_below_limit').value
        self._proc = None  # current audio subprocess

        if self._audio_above:
            self.get_logger().info(f'Audio ACIMA : {self._audio_above}')
        if self._audio_below:
            self.get_logger().info(f'Audio ABAIXO: {self._audio_below}')
        if not self._audio_above and not self._audio_below:
            self.get_logger().info('Sem arquivos de audio — usando espeak-ng TTS.')

        self.create_service(SetBool, 'play_audio', self._handle)
        self.get_logger().info('Audio Alert: service /play_audio pronto.')

    def _handle(self, request, response):
        # Drop request if previous audio is still playing
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().warn('Audio ainda em reproducao — requisicao ignorada.')
            response.success = False
            response.message = 'already playing'
            return response

        is_above = request.data
        label = 'ACIMA DO LIMITE' if is_above else 'DENTRO DO LIMITE'
        self.get_logger().info(f'[alerta] Pressao: {label}')

        audio_file = self._audio_above if is_above else self._audio_below

        if audio_file and os.path.isfile(audio_file):
            self._proc = subprocess.Popen(
                ['mpg123', '-q', audio_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            text = ('pressao acima do limite' if is_above
                    else 'pressao dentro do limite')
            self._proc = subprocess.Popen(
                ['espeak-ng', '-v', 'pt', text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        response.success = True
        response.message = 'playing'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AudioAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
