/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2    
 * LAST UPDATE: 9/8/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the right-hand rule.
 */

#include "student.h"

// Ignoring this line until project 5
turtleMove studentTurtleStep(bool bumped) {
    return MOVE;
}

// Replacing #define with a constant variable
const int TIMEOUT = 40;  // Timer value to slow down the simulation for better visibility

// Typedefs for readability and future flexibility
typedef int State;    // Typedef for state representation
typedef int Position; // Typedef for position coordinates
typedef bool Flag;    // Typedef for boolean flags

// Enum to represent directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

/**
 * @brief Checks the turtle's direction and updates its orientation and state.
 * 
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * @param bumpedFlag  Indicates if the turtle has bumped into an obstacle.
 * @param currentState The current state of the turtle's movement.
 * 
 * @details
 * Purpose: This function determines the turtle's next move based on its current orientation and whether it has bumped into an obstacle. 
 *          It updates the orientation and state of the turtle to ensure it follows the right-hand rule in navigating the maze.
 * Inputs:  - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 *          - `bumpedFlag`: A boolean indicating if the turtle has encountered an obstacle.
 *          - `currentState`: An integer representing the turtle's current state.
 * Outputs: - Updates the `orientation` variable to reflect the turtle's new direction.
 *          - Updates the `currentState` to control the turtle's movement logic.
 * Saved Internal: Updates the `currentState` to manage state transitions.
 */
void checkDirection(int& orientation, Flag bumpedFlag, State& currentState) {
    switch (orientation) {
        case NORTH:
            if (currentState == 2) {
                orientation = EAST;  // Turn right to face East
                currentState = 1;
            } else if (bumpedFlag) {
                orientation = WEST;  // Turn left to face West if bumped
                currentState = 0;
            } else {
                currentState = 2;    // Move forward if no bump
            }
            break;
        case EAST:
            if (currentState == 2) {
                orientation = SOUTH; // Turn right to face South
                currentState = 1;
            } else if (bumpedFlag) {
                orientation = NORTH; // Turn left to face North if bumped
                currentState = 0;
            } else {
                currentState = 2;    // Move forward if no bump
            }
            break;
        case SOUTH:
            if (currentState == 2) {
                orientation = WEST;  // Turn right to face West
                currentState = 1;
            } else if (bumpedFlag) {
                orientation = EAST;  // Turn left to face East if bumped
                currentState = 0;
            } else {
                currentState = 2;    // Move forward if no bump
            }
            break;
        case WEST:
            if (currentState == 2) {
                orientation = NORTH; // Turn right to face North
                currentState = 1;
            } else if (bumpedFlag) {
                orientation = SOUTH; // Turn left to face South if bumped
                currentState = 0;
            } else {
                currentState = 2;    // Move forward if no bump
            }
            break;
    }
}

/**
 * @brief Updates the position of the turtle based on its current orientation.
 * 
 * @param position    Current position of the turtle as a QPointF object.
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * 
 * @details
 * Purpose: This function updates the turtle's position on the grid based on its current orientation. The position is adjusted to simulate movement in the specified direction.
 * Inputs:  - `position`: A QPointF object representing the current position of the turtle on the grid.
 *          - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 * Outputs: - Modifies the `position` variable to reflect the turtle's new location on the grid.
 * Saved Internal: None.
 */
void updatePosition(QPointF& position, int orientation) {
    switch (orientation) {
        case EAST:
            position.setY(position.y() - 1); // Move East (right)
            break;
        case SOUTH:
            position.setX(position.x() + 1); // Move South (down)
            break;
        case WEST:
            position.setY(position.y() + 1); // Move West (left)
            break;
        case NORTH:
            position.setX(position.x() - 1); // Move North (up)
            break;
    }
}

/**
 * @brief Determines whether the turtle should move and updates its position accordingly.
 * 
 * @param position    Current position of the turtle as a QPointF object.
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * @return true if changes should be submitted, false otherwise.
 * 
 * @details
 * Purpose: This function controls the turtle's movement within the maze, determining if it should move based on the current timer, its orientation, and whether it has reached the end of the maze.
 *          It manages the movement logic by updating the position and orientation of the turtle while considering potential obstacles.
 * Inputs:  - `position`: A QPointF object representing the current position of the turtle on the grid.
 *          - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 * Outputs: - Returns `true` if the turtle's movement should be submitted (timer has reset), or `false` if it should not.
 * Saved Internal: - Updates the local variables `timer`, `currentState`, `shouldMove`, `atEnd`, `modifyFlag`, and `bumpedFlag` to manage the turtle's state and logic flow.
 */
bool studentMoveTurtle(QPointF& position, int& orientation) {
    // Define all variables at the start of the procedure
    static int timer = TIMEOUT;        // Timer for managing movement
    static State currentState = 0;     // Current state of the turtle's movement
    Position futureX1, futureY1, futureX2, futureY2; // Future positions based on orientation
    Flag shouldMove = false;            // Flag to determine if turtle should move
    Flag atEnd = false;                 // Flag to check if turtle has reached the end
    Flag modifyFlag = true;             // Flag to check if movement needs modification
    Flag bumpedFlag = false;            // Flag to check if turtle bumped into something

    ROS_INFO("Turtle update called - timer=%d", timer);

    // Timer countdown logic
    if (timer == 0) { // Timer has completed its countdown, execute logic
        // Initialize future positions
        futureX1 = position.x();
        futureY1 = position.y();
        futureX2 = position.x();
        futureY2 = position.y();

        // Determine the future position based on the current orientation
        switch (orientation) {
            case NORTH:
                futureY2 += 1; // Moving North increases Y
                break;
            case EAST:
                futureX2 += 1; // Moving East increases X
                break;
            case SOUTH:
                futureX2 += 1; // Moving South increases X
                futureY2 += 1; // Moving South increases Y (diagonal)
                futureX1 += 1;
                break;
            case WEST:
                futureX2 += 1; // Moving West increases X
                futureY2 += 1; // Moving West increases Y (diagonal)
                futureY1 += 1;
                break;
        }

        // Check if the turtle is about to bump into a wall or has reached the end
        bumpedFlag = bumped(futureX1, futureY1, futureX2, futureY2);
        atEnd = atend(position.x(), position.y());

        // Check direction and update orientation
        checkDirection(orientation, bumpedFlag, currentState);

        ROS_INFO("Orientation=%d  STATE=%d", orientation, currentState);

        shouldMove = (currentState == 2);
        modifyFlag = true;

        // Move the turtle if allowed and not at the end
        if (shouldMove && !atEnd) {
            updatePosition(position, orientation);
            shouldMove = false;
            modifyFlag = true;
        }
    }

    // Check if the turtle has reached the end of the maze
    if (atEnd) return false;

    // Update the timer
    timer = (timer == 0) ? TIMEOUT : timer - 1;

    // Submit changes if the timer has reset
    return (timer == TIMEOUT);
}
