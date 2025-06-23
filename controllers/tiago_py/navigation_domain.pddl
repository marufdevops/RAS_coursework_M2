(define (domain navigation)
  (:requirements :strips :typing)
  
  (:types
    robot node - object
  )
  
  (:predicates
    (at ?robot - robot ?node - node)
    (connected ?from - node ?to - node)
  )
  
  (:action move
    :parameters (?robot - robot ?from - node ?to - node)
    :precondition (and 
      (at ?robot ?from)
      (connected ?from ?to)
    )
    :effect (and 
      (not (at ?robot ?from))
      (at ?robot ?to)
    )
  )
)