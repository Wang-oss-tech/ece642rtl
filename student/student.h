#ifndef STUDENT_H
#define STUDENT_H

#include <ros/ros.h>
#include <QPointF>  // Assuming you're using Qt for handling coordinates

// ROS interaction functions. Do not change these lines.
bool bumped(int x1, int y1, int x2, int y2);   // Check if there's a bump
bool atend(int x, int y);                      // Check if the turtle is at the end of the maze
void displayVisits(int visits);                // Display number of visits to a location in the GUI
bool moveTurtle(QPointF& pos_, int& nw_or);    // Move the turtle in the maze

// Turtle movement enumeration
enum turtleMove {
    MOVE_FORWARD,  // Command to move forward
    TURN_LEFT,     // Command to turn left
    TURN_RIGHT     // Command to turn right
};

// Function to handle the decision-making in student_turtle.cpp.
// Determines the next turtle move based on whether a bump has occurred.
turtleMove studentTurtleStep(bool bumped);

// Position translation functions for converting relative movements into absolute coordinates.
// Implemented in student_maze.cpp
QPointF translatePos(QPointF pos_, int orientation, turtleMove nextMove); // Translate relative movement to absolute position
int translateOrnt(int orientation, turtleMove nextMove);                  // Translate relative orientation to absolute orientation

// Declaration for visit tracking functions in student_maze.cpp
void incrementVisits(int32_t x, int32_t y);  // Increment the number of visits to a particular cell
int32_t getVisits(int32_t x, int32_t y);     // Get the number of visits to a particular cell

// Function to check if the right-hand side is clear, implemented in student_maze.cpp
bool rightIsClear(QPointF pos_, int orientation);  // Check if the turtle's right-hand side is clear

#endif  // STUDENT_H
