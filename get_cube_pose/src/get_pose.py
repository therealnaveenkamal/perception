import rclpy
from rclpy.action import ActionClient
from grasping_msgs.action import FindGraspableObjects

class FindObjectsClient:
    def __init__(self):
        self.node = rclpy.create_node('find_objects_client')
        self.client = ActionClient(self.node, FindGraspableObjects, 'find_objects')

        # Wait for the action server to become available
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error('Action server not available after waiting')
            self.node.destroy_node()
            return

        self.send_goal()

    def send_goal(self):
        goal_msg = FindGraspableObjects.Goal()
        goal_msg.plan_grasps = False

        self.node.get_logger().info('Sending goal')

        send_goal_future = self.client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, send_goal_future)

        if send_goal_future.result() is not None:
            goal_handle = send_goal_future.result()
            result_future = goal_handle.get_result_async()

            # Wait for the result
            rclpy.spin_until_future_complete(self.node, result_future)

            if result_future.result() is not None:
                result = result_future.result()
                self.handle_result(result)
            else:
                self.node.get_logger().error('Failed to get result')
        else:
            self.node.get_logger().error('Failed to send goal')

        self.node.destroy_node()

    def handle_result(self, result):
        print(result.result.support_surfaces, file=open('/home/user/ros2_ws/src/manipulation_project/get_cube_pose/src/get_pose_logfile.txt', 'w'))
        if len(result.result.objects[0].object.primitives) == 1:
            print('Goal succeeded')
            print(result.result.support_surfaces[0].primitive_poses)
            #self.node.get_logger().info('Y: %f', result.result.objects[0].object.primitive_poses[0].position.y)
        else:
            self.node.get_logger().error('Goal failed with code')


def main():
    rclpy.init()
    find_objects_client = FindObjectsClient()    

if __name__ == '__main__':
    main()
