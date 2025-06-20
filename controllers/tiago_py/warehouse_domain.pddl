(define (domain warehouse-navigation)
  (:requirements :typing :strips)
  
  (:types
    location - object
    room - location
    target - location
    robot - object
  )
  
  (:predicates
    ;; Robot location predicates
    (robot-at ?r - robot ?l - location)
    
    ;; Connectivity predicates
    (connected ?from - location ?to - location)
    
    ;; Room membership predicates
    (in-room ?loc - location ?room - room)
    
    ;; Target accessibility predicates
    (target-accessible ?t - target)
    
    ;; Navigation state predicates
    (path-clear ?from - location ?to - location)
    (robot-oriented ?r - robot ?l - location)
  )
  
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (robot-at ?r ?from)
      (connected ?from ?to)
      (path-clear ?from ?to)
    )
    :effect (and
      (robot-at ?r ?to)
      (not (robot-at ?r ?from))
      (robot-oriented ?r ?to)
    )
  )
  
  (:action navigate-to-room
    :parameters (?r - robot ?from - location ?room - room ?to - location)
    :precondition (and
      (robot-at ?r ?from)
      (in-room ?to ?room)
      (connected ?from ?to)
      (path-clear ?from ?to)
    )
    :effect (and
      (robot-at ?r ?to)
      (not (robot-at ?r ?from))
      (robot-oriented ?r ?to)
    )
  )
  
  (:action approach-target
    :parameters (?r - robot ?from - location ?target - target)
    :precondition (and
      (robot-at ?r ?from)
      (target-accessible ?target)
      (connected ?from ?target)
      (path-clear ?from ?target)
    )
    :effect (and
      (robot-at ?r ?target)
      (not (robot-at ?r ?from))
      (robot-oriented ?r ?target)
    )
  )
)
