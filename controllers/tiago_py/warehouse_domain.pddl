(define (domain warehouse)
    (:requirements :strips)
    (:types
        robot waypoint - object
    )
    (:predicates
        (at ?r - robot ?w - waypoint)
        (connected ?w1 - waypoint ?w2 - waypoint)
    )
    (:action move
        :parameters (?r - robot ?from - waypoint ?to - waypoint)
        :precondition (and
            (at ?r ?from)
            (connected ?from ?to)
        )
        :effect (and
            (not (at ?r ?from))
            (at ?r ?to)
        )
    )
)