# MRS Summer School 2020

## Supervision node for the Leader-Follower task

This node will keep track of the score for the task. The score is accumulated per time, if the UAVs maintain visual contact. The score counter will automatically terminate once the visual link is broken for longer period than allowed by the competition rules.

Scoring:
  * 1 point for every 10 ms in contact
  * leader will follow an increasingly difficult trajectory, and its velocity will grow
