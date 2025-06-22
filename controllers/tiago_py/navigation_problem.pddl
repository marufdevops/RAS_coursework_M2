(define (problem navigation-to-balls)
  (:domain navigation)

  (:objects
    tiago - robot
    node1
    node2
    node3
    node4
    node5
    node6 - node
  )

  (:init
    (at tiago node1)
    (connected node1 node2)
    (connected node2 node1)
    (connected node2 node3)
    (connected node3 node2)
    (connected node3 node4)
    (connected node4 node3)
    (connected node2 node5)
    (connected node5 node2)
    (connected node5 node6)
    (connected node6 node5)
    (connected node5 node4)
    (connected node4 node5)
    (connected node4 node6)
    (connected node6 node4)
  )

  (:goal
    (at tiago node4)
  )
)