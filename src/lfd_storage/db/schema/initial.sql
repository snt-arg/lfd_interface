-- Create a database for the robot demonstration framework
CREATE DATABASE robot_demos;

-- Switch to the robot_demos database
\c robot_demos;

-- Table to store metadata about robots
CREATE TABLE robots (
    id SERIAL PRIMARY KEY,                            -- Unique identifier for each robot
    name TEXT NOT NULL,                               -- Name of the robot
    config JSONB,                                     -- Configuration details stored as JSON
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP    -- Timestamp for when the robot was added
);

-- Table to store metadata about demonstrations
CREATE TABLE demonstrations (
    id SERIAL PRIMARY KEY,                -- Unique identifier for each demonstration
    name TEXT NOT NULL,                   -- Name of the robot demonstration
    robot_id INT NOT NULL,                -- Foreign key referencing robots
    meta_data JSONB,                       -- Metadata stored as JSON (e.g., task info)
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,   -- Timestamp for when the demonstration was created
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE CASCADE
);

-- Table to store trajectories related to demonstrations
CREATE TABLE trajectories (
    id SERIAL PRIMARY KEY,               -- Unique identifier for each trajectory
    demo_id INT NOT NULL,                -- Foreign key referencing demonstrations
    type VARCHAR(50) NOT NULL,           -- Type of trajectory (raw, filtered, smoothed, refined, reversed)
    trajectory_data BYTEA NOT NULL,      -- Trajectory data stored as a pickle
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,  -- Timestamp for when the trajectory was created
    FOREIGN KEY (demo_id) REFERENCES demonstrations(id) ON DELETE CASCADE
);

-- Example indexes for faster querying
CREATE INDEX idx_robot_name ON robots(name);
CREATE INDEX idx_demo_name ON demonstrations(name);
CREATE INDEX idx_trajectory_type ON trajectories(type);
