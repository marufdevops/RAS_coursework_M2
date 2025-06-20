(define (problem warehouse-navigation)
    (:domain warehouse)
    (:objects
        robot - robot
        start center_west center_east north_west north_east south_west south_east
        red_approach green_approach ducks_approach balls_approach
        west_corridor east_corridor north_central south_central - waypoint
    )
    (:init
        (at robot start)
        ; Waypoint connections (bidirectional) - based on actual warehouse layout
        ; Start connections
        (connected start center_west)
        (connected center_west start)
        (connected start south_west)
        (connected south_west start)
        
        ; Central area connections (around the central wall)
        (connected center_west center_east)
        (connected center_east center_west)
        (connected center_west north_west)
        (connected north_west center_west)
        (connected center_west south_west)
        (connected south_west center_west)
        (connected center_east north_east)
        (connected north_east center_east)
        (connected center_east south_east)
        (connected south_east center_east)
        
        ; North-south corridor connections
        (connected north_west north_central)
        (connected north_central north_west)
        (connected north_east north_central)
        (connected north_central north_east)
        (connected south_west south_central)
        (connected south_central south_west)
        (connected south_east south_central)
        (connected south_central south_east)
        
        ; East-west corridor connections
        (connected center_west west_corridor)
        (connected west_corridor center_west)
        (connected center_east east_corridor)
        (connected east_corridor center_east)
        
        ; Target approach connections
        (connected north_east red_approach)
        (connected red_approach north_east)
        (connected east_corridor red_approach)
        (connected red_approach east_corridor)
        
        (connected south_east green_approach)
        (connected green_approach south_east)
        (connected east_corridor green_approach)
        (connected green_approach east_corridor)
        
        (connected north_west ducks_approach)
        (connected ducks_approach north_west)
        (connected west_corridor ducks_approach)
        (connected ducks_approach west_corridor)
        
        (connected south_west balls_approach)
        (connected balls_approach south_west)
        (connected west_corridor balls_approach)
        (connected balls_approach west_corridor)
    )
    (:goal
        (at robot red_approach)
    )
)
