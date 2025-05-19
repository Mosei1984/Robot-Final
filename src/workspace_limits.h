#pragma once

#include <Arduino.h>

// Define workspace limits for the robot
namespace WorkspaceLimits {
    // Constants for workspace limits based on physical robot dimensions
    const float WORKSPACE_MIN_X = -160.0f;  // Minimum X coordinate (mm)
    const float WORKSPACE_MAX_X =  160.0f;  // Maximum X coordinate (mm)
    const float WORKSPACE_MIN_Y = -160.0f;  // Minimum Y coordinate (mm)
    const float WORKSPACE_MAX_Y =  160.0f;  // Maximum Y coordinate (mm)
    const float WORKSPACE_MIN_Z =   50.0f;  // Minimum Z coordinate (mm)
    const float WORKSPACE_MAX_Z =  210.0f;  // Maximum Z coordinate (mm)
    
    const float WORKSPACE_MIN_YAW   = -PI;     // Minimum yaw angle (rad)
    const float WORKSPACE_MAX_YAW   =  PI;     // Maximum yaw angle (rad)
    const float WORKSPACE_MIN_PITCH = -PI/2;   // Minimum pitch angle (rad)
    const float WORKSPACE_MAX_PITCH =  PI/2;   // Maximum pitch angle (rad)
    const float WORKSPACE_MIN_ROLL  = -PI;     // Minimum roll angle (rad)
    const float WORKSPACE_MAX_ROLL  =  PI;     // Maximum roll angle (rad)
    
    // Structure to define a 3D box in space
    struct BoundingBox {
        float minX, maxX;
        float minY, maxY;
        float minZ, maxZ;
    };
    
    // Function to check if a position is within the workspace
    inline bool isPositionReachable(float x, float y, float z) {
        // Basic boundary check
        if (x < WORKSPACE_MIN_X || x > WORKSPACE_MAX_X ||
            y < WORKSPACE_MIN_Y || y > WORKSPACE_MAX_Y ||
            z < WORKSPACE_MIN_Z || z > WORKSPACE_MAX_Z) {
            return false;
        }
        
        // You could add more complex checks here like radial distance
        // For now we'll use a simple cubic workspace
        return true;
    }
    
    // Function to get the default workspace bounding box
    inline BoundingBox getWorkspaceBoundingBox() {
        BoundingBox box;
        box.minX = WORKSPACE_MIN_X;
        box.maxX = WORKSPACE_MAX_X;
        box.minY = WORKSPACE_MIN_Y;
        box.maxY = WORKSPACE_MAX_Y;
        box.minZ = WORKSPACE_MIN_Z;
        box.maxZ = WORKSPACE_MAX_Z;
        return box;
    }
    
    // Function to calculate a safe height for movements above the workspace
    inline float getSafeHeight() {
        return WORKSPACE_MAX_Z - 20.0f; // 20mm below max height
    }
}
