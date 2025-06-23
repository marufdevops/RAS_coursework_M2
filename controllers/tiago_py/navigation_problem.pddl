(define (problem navigation-to-red)
  (:domain navigation)

  (:objects
    tiago - robot
    node1
    node2
    node3
    node4
    node5
    node6
    node7
    node8
    node9
    node10
    node11 - node
  )

  (:init
    (at tiago node6)
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
    (connected node1 node7)
    (connected node7 node1)
    (connected node7 node8)
    (connected node8 node7)
    (connected node8 node9)
    (connected node9 node8)
    (connected node2 node8)
    (connected node8 node2)
    (connected node8 node10)
    (connected node10 node8)
    (connected node9 node10)
    (connected node10 node9)
    (connected node10 node11)
    (connected node11 node10)
    (connected node11 node6)
    (connected node6 node11)
    (connected node5 node11)
    (connected node11 node5)
  )

  (:goal
    (at tiago node9)
  )
)