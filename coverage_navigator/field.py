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
    print("Please input the points for the field (enter 'done' when finished):")
    while True:
        # Ask the user for the next point
        point_input = input("Enter a point as 'x y' or 'done' to finish: ")

        # If the user inputs 'done', stop the loop
        if point_input.lower() == 'done':
            break

        try:
            # Parse the input into x and y coordinates
            point = list(map(float, point_input.split()))
            # Check that there are 2 points
            if len(point) != 2:
                print("Error: You must enter exactly two values for x and y.")
                continue
            # Add the point to the field list
            field.append(point)
        except ValueError:
            print("Error: Invalid input. Please enter two numeric values separated by space.")

    if len(field) < 3:
        print("Error: A valid polygon requires at least 3 points.")
        return

    # Ensure the polygon is closed (i.e., the last point should be the same as the first)
    if field[0] != field[-1]:
        field.append(field[0])

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