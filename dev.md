# Development Notes

## Could be bugs

- Pelorus dbg timeout receiving ack only waiting or not receiving?

## Definitely bugs

- [ratpub] Long polling subscribers are not cancelled when subscriber disconnects
- [ratpub/wind] Sending huge initial wind seems to break the sail
- [ratpub] Should be easier to use inside smol-rs runtime

## Not supported features interpreted as bugs that are not planned to be solved in Minot

- [ratpub] When a subscriber is in long polling process and another publisher joins on the same topic, the subscriber does not get messages from the new publisher but the publisher wants to send it to the old publisher. (Can be solved in combination with cancelling.)


# Backlog

1. Compare: Goto does nothing
1. Try ROS1 shared wind trigger and compare matrix
