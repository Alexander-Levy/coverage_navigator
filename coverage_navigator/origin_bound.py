import time
import rclpy

from coverage_navigator.coverage_navigator import CoverageNavigator, TaskResult

def main():

    # Initialize the node
    rclpy.init()

    # Initialize the coverage navigator
    navigator = CoverageNavigator()
    navigator.waitNavStartup() # wait for the navigation stack to be active

    # Create an empty list to store the field coordinates
    field = []

    # Asks for user input
    print("Please input the bounds for the field (staring at [0, 0])")
    bound_input = input("Enter the bounds of the field: ")
    field = [[0.0, 0.0], [0.0, bound_input], [bound_input, bound_input], [bound_input, 0.0], [0.0, 0.0]]

    # Now call the navigateCoverage function with the user-provided field
    navigator.navigateCoverage(field)

    i = 0
    while not navigator.isTaskComplete():
        # Print message while navigating
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Navigating selected area...')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Coverage completed!')
    elif result == TaskResult.CANCELED:
        print('Coverage action was cancelled!')
    elif result == TaskResult.FAILED:
        print('Coverage failed!')
    else:
        print('Goal has an invalid return status!')