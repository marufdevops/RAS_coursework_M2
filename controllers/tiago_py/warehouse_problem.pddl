(define (problem navigate-to-balls)
    (:domain warehouse)
    (:objects
        robot - robot
        start intermediate balls_approach - waypoint
    )
    (:init
        (at robot start)
        ; Simple waypoint connections
        (connected start intermediate)
        (connected intermediate start)
        (connected intermediate balls_approach)
        (connected balls_approach intermediate)
    )
    (:goal
        (at robot balls_approach)
    )
)