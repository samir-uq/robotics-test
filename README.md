
# FRC 3748

## 2025 Reefscape Codebase


<!-- ### Disclaimer: The Robot Code is subject to change heavily for the 2026 season, this is a reference to get into FRC programming for 3748 -->

## Robot Features

1. Autonomous Mode
2. Coral L2 / L3 Telescopic Arm
3. Algae Ground Intake / Processor (Replaced)
4. L1 Ground Intake
5. Dealigfy
6. Drivetrain
7. Vision


## Detailed Robot Features

### 1. Autonomous Mode

Due to the lack of L4 functionality, we had to compromise heavily on autonomous mode.
Touching the Algae would result in the coral points not counting, therefore, branches where Algae was on the lower (L2) area would be invalid and not count as a proper Auto point.

Alongside, our Dealgify mechanism being very simplisitc and lacking precise control, we were not able to add this in our auto mode.

This leaves us to only being able to score in branches where the Algae is at the top (L3). This would be 2 o clock, 6 o clock, 10 o clock branches. The clock system is based off the perspective of the driver through the driver station.

With such constraints and suboptimized drivetrain, we were able to only create a two piece L2 auto.

For 2025 season, our entire auto was inside of FRC PathPlanner, which let us build paths, create autos and run it.
In order for PathPlanner to execute costumized tasks based on our robot, we need to input a lot of variables.


This code sets up our autobuilder, which allows us to run all drive train based actions. This can be found Under Drive.java



```java
 AutoBuilder.configure(
                this::getPose,
                this::resetPoseEstimator,
                this::getChassisSpeeds,
                this::setPathPlannerRobotSpeeds,
                new PPHolonomicDriveController(
                        Constants.Auto.xyController, Constants.Auto.thetaController),
                Constants.Auto.ppConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
```

For further details, this sends the method that gets the pose of the drivetrain, the method that resets the estimator, method that gets chassisSpeeds and method that sets PathPlannerRobotSpeeds. this syntax of Java is very important to code for FRC, as most of the code revolves around using the WPILIB library.

The previous code block only handles the drive train, which might be a key part of Auto, but is not all. Inside of RobotContainer.java, there is a method that is named

```java
 private void configureNamedCommands() {...}
 ```
 Inside of this method, we declare a name for a Command, which will be executed as described in FRC Pathplanner.

 To explain it in simpler terms, in PathPlanner, we will have a list of Commands that will be executed at different setpoints. PathPlanner will look for these named commands and execute the Command Object that we also send within the declaration.

 To understand this even better, you need to have a basic understanding of what Commands are in WPILIB.

 #### Commands

 A Command is an Object in Java that has a set of methods that gives a robot specific actions to do. This command can be scheduled in different subsystems, which we will touch more on later through the documentation. 

 For example, for the driver to properly drive the robot in teleop, the command to set the speed has to be scheduled. Every "frame", the robot gets the value of the joysticks and utilizes that to generate the proper speed the robot should go to and sets it as such. This command is scheduled every frame, which allows for smooth fluidity and full control of the robot. More of this could be found on DriveManual.java

Overall, Commands, PathPlanner and Drivetrain play a huge part in creating a successful auto. Although PathPlanner will not be utilized for 2026 season, it is still used heavily and gives us great control over autonomous mode.

A command has many methods.

```java
    // Constructor
    public Command(...) {...}

    /// is called whenever the command is initialized to run
    /// important to note that you will send a Command object through the parameters. This means that initialize will be called multiple times in the same object, not just once.
    public void initialize() {...}

    // execute is called per "frame" until the code is over
    public void execute() {...}

    //  isFinished is also called per "frame" after execute to see if the command has achieved its goal and could end
    public boolean isFinished() {...}

    // end is called at the end of the command finishing.
    // this could either be because the command was canceled or it was completed
    public void end(boolean interupted) {...}

```

### 2. Coral L2 / L3 Telescopic Arm

To fully understand how this feature is developed, you need to have a deep understanding about what a subsystem is in FRC.

#### Subsystem

Think of subsystem like an individual component of the robot, it is seperated into its own part. You can see within the codebase (folder Subsystems), there are different parts of the robot that are programmed differently. 

The subsystem then acts as the central point for controlling a mechanical part of the robot. The subsystem for the Arm has multiple methods that allows us to set its position.

```java
 public void setElevatorPosition(double height) {...}
 ```
 This is apart of the Elevator subsystem, which is also called the Arm.
 

Three main things of a subsystem are

```java
// constructor to initialize the class
public Subsystem(...) {...}


// this is called per frame so the subsystem can always update.
public void periodic() {...}

// this next one is a bit complicated so there will be an example instead of an interface example

// Basically. each state has its own logged state from AutoLog, which is a library we use to log different variables and test to make sure our robots are fine. AutoLog only support primitives and some WPILIB classes. It is very useful and should be utilized for EVERY subsystem.

    @Override
    public ElevatorStatesAutoLogged updateStates(ElevatorStatesAutoLogged _elevatorStates) {

        _elevatorStates.OutputCurrent = elevator.getOutputCurrent();
        _elevatorStates.Temp = elevator.getMotorTemperature();

        _elevatorStates.elevatorPosition = getElevatorPosition();
        _elevatorStates.elevatorSpeed = getElevatorSpeed();

        return _elevatorStates;
    }

```

Last but not least, you can also see that all subsystem have [SubSystemName]IO.java. This is called an interface, which is a java feature that every programmer should know and utilize.

In future seasons, the plan is to add [SubSystemName]Sim.java, which allows us to fully simulate the robot so we can test it at home and make sure everything is perfect. Exampels of this could be found on teams that like 422.

##### Now. Whats the difference between a Subsystem and a Command

A command is utilized to perform a singular task, or a set of tasks, whilst a subsystem is utilized to control a component. For example, if we want the arm to be set at L2 position, we would have a command that executes the setElevatorPosition from the subsystem.

This way, most of our commands just help create setpoints which is utilized by the drivers to efficiently score.

Returning back to the Arm, it is not very complex.
THe arm just rises up and down and the code just controls how the motors spin so it will be at the right position. As you look through WPILIB, you can see how important PIDs and other classes are to creating a fine robot.

### 3,4,5

Not to get lazy and not explain each subsystem in detail, but they all work similarly. They utilize motors and help make sure things are at the correct setpoint.

Throughout the offseason, the code team from 2025 will create insightful comments on all of the files to make sure everybody has a clear understanding of how the robot was programmed at each step. This documentation just helps provide the basic information.

### 6. Drivetrain

Our drivetrain is swerve drive. They are from West Coast Products and are utilized by teams like MadTown Robotics, who have had a great season. Our drivetrain includes a lots of things and should be looked over individually to get a grasp. Lots of classes from WPILIB and other FRC related libraries are utilized and can be searched online. There are also many online informations on how swerve drive works. Part of being a programmer means having to do great research and drive train is one of them. They are the heart of every robot and it is a must to have a great drive train to even be a decent robot.

As for this season, AutoAlign was utilized to make sure the robot can drive itself onto the branch so it would be very easy to place the Corals. This posed a lot of problems early on the season as the code team did not properly know how to set our Vision up. But as we went further, we looked at other teams and took their code, changed it, to make something that was viable for us.

### 7. Vision

For our vision, we use 2 Limelights. Limelight has great documentation and is very easy to use. They handle all the processing and just return us values, which we sort a little and handpick to make sure we are not blindly trusting every data we recieve. More of Vision could also be found on the internet.

Vision is very vital, therefore, there needs to be a greater emphasis on it in the future years. luckily, with the help of limelights, we were able to pull off a sucessfull pose estimation for aligning. Which again, is all done by the limelight internally. Therefore, there is not much to explain code wise for now, it is better to research on your own and see how they function.

With all that being said, there needs to be a lot of changes for vision next year, it is always recommended to see how other teams handle vision to make sure we are not doing something outlandish or stupid.

## Issues and Fixes

At the start of the season, the team was really lost and had difficulties setting up AutoAlign to make sure robot would be directly at the branch to score the coral properly. To help solve this problem and create an efficient robot, much research was done by looking at other great teams robot code. This is heavily encouraged, the more you look at other peoples code, the more flaws you detect in yours, which prompts you to write better code and have a better robot.

Alongside, our drivetrain had a lot of problems, especially with odometry, as it was not that accurate. This caused us to rely on vision a lot and had some issues whilst making the auto paths. Although the error ratio was minimized, the team was not able to fully figure out why this was.

As for the subsytems, the team was not able to fully incorporate MaxMotion on our arm to make it faster. This caused our robot to not be that fast and created lower cycle time.

In the field, after auto is complete, our robot's gyro would zero, causing the driver to have messed up controls. Unable to properly identify why and fix it, we always lost the first 3-5 seconds of teleop to fix this issue.

As for our vision, going really fast would cause the Limelight to see the tag with high error percentile, making the readings more innacurate and causing our robot to think it is in a different pose. Although not a big issue, this made it so we were not able to fully rely on pose to make our robot more faster and code heavy like the other teams. This feature is very important for future years as it is the heart of having a great robot. Great Align, Code Heavy Drivetrain and Being Dynamic.


## 2026

For 2026, the plan is to recreate the robot code from the ground up with a new way of programming. Looking into great teams by 422 and 2910, we should draw huge inspirations to make our robots like them code wise. This could possibly allow us to have a great functioning robot every year.

This starts with redoing the code for 2025 robot fully in the offseason. This documentation and code stands as an archive for what was achieved in the 2025 season but it should not be the standard going forward.

## To Get Started

Go to Main.java to start on line by line documentation
View each file and understand the logic.