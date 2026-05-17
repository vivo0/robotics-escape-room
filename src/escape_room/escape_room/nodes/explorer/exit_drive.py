"""Exit phase: drive forward through the door opening (which lies in
unmapped space that Nav2 won't plan into)."""

from geometry_msgs.msg import Twist


def tick_exit_drive(node) -> None:
    if node.clock_s() - node.action_t >= node.exit_drive_duration:
        node.stop()
        node.enter("done")
        return
    twist = Twist()
    twist.linear.x = node.exit_drive_speed
    node.cmd_pub.publish(twist)
