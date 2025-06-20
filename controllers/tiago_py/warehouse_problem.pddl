(define (problem warehouse-navigation-task)
  (:domain warehouse-navigation)
  
  (:objects
    ;; Robot
    tiago - robot
    
    ;; Rooms
    left-room - room
    right-room - room
    center-corridor - room
    
    ;; Navigation waypoints
    start-pos - location
    left-upper - location
    left-lower - location
    right-upper - location
    right-lower - location
    corridor-center - location
    corridor-north - location
    corridor-south - location
    
    ;; Target locations
    red-box - target
    green-box - target
    ducks-container - target
    balls-container - target
  )
  
  (:init
    ;; Initial robot position
    (robot-at tiago start-pos)
    
    ;; Room membership
    (in-room left-upper left-room)
    (in-room left-lower left-room)
    (in-room right-upper right-room)
    (in-room right-lower right-room)
    (in-room corridor-center center-corridor)
    (in-room corridor-north center-corridor)
    (in-room corridor-south center-corridor)
    
    ;; Target room assignments
    (in-room ducks-container left-room)
    (in-room balls-container left-room)
    (in-room red-box right-room)
    (in-room green-box right-room)
    
    ;; Connectivity - bidirectional paths
    ;; Start position connections
    (connected start-pos corridor-center)
    (connected corridor-center start-pos)
    
    ;; Corridor internal connections
    (connected corridor-center corridor-north)
    (connected corridor-north corridor-center)
    (connected corridor-center corridor-south)
    (connected corridor-south corridor-center)
    
    ;; Left room connections
    (connected corridor-north left-upper)
    (connected left-upper corridor-north)
    (connected corridor-south left-lower)
    (connected left-lower corridor-south)
    (connected left-upper left-lower)
    (connected left-lower left-upper)
    
    ;; Right room connections
    (connected corridor-north right-upper)
    (connected right-upper corridor-north)
    (connected corridor-south right-lower)
    (connected right-lower corridor-south)
    (connected right-upper right-lower)
    (connected right-lower right-upper)
    
    ;; Target accessibility
    (connected left-upper ducks-container)
    (connected ducks-container left-upper)
    (connected left-lower balls-container)
    (connected balls-container left-lower)
    (connected right-upper red-box)
    (connected red-box right-upper)
    (connected right-lower green-box)
    (connected green-box right-lower)
    
    ;; All targets are accessible initially
    (target-accessible red-box)
    (target-accessible green-box)
    (target-accessible ducks-container)
    (target-accessible balls-container)
    
    ;; All paths are clear initially
    (path-clear start-pos corridor-center)
    (path-clear corridor-center start-pos)
    (path-clear corridor-center corridor-north)
    (path-clear corridor-north corridor-center)
    (path-clear corridor-center corridor-south)
    (path-clear corridor-south corridor-center)
    (path-clear corridor-north left-upper)
    (path-clear left-upper corridor-north)
    (path-clear corridor-south left-lower)
    (path-clear left-lower corridor-south)
    (path-clear left-upper left-lower)
    (path-clear left-lower left-upper)
    (path-clear corridor-north right-upper)
    (path-clear right-upper corridor-north)
    (path-clear corridor-south right-lower)
    (path-clear right-lower corridor-south)
    (path-clear right-upper right-lower)
    (path-clear right-lower right-upper)
    (path-clear left-upper ducks-container)
    (path-clear ducks-container left-upper)
    (path-clear left-lower balls-container)
    (path-clear balls-container left-lower)
    (path-clear right-upper red-box)
    (path-clear red-box right-upper)
    (path-clear right-lower green-box)
    (path-clear green-box right-lower)
  )
  
  ;; Goal will be set dynamically based on user input
  (:goal (robot-at tiago red-box))
)
