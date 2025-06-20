(define (domain warehouse)
    (:requirements :typing)
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
            (at ?r ?to)
            (not (at ?r ?from))
        )
    )
)
