# IconHwmController reads this to learn the robot status.
#
# For best error handling / recovery support, implement a node that 
# * publishes OperationalStatus
# * implements an std_srvs/srv/Trigger service for clearing faults

# state constants
uint8 UNKNOWN = 0
# Connected to the robot, receiving cyclic status values, 
# but not sending commands (robot not moving)
uint8 DISABLED = 1
# Robot is in an error state, and *at least* requires acknowledgment
# of the fault via the clear_faults Trigger service.
#
# If `state` is `FAULTED`, `message` should contain details about the
# error and how to fix it.
# A successful call to the clear_faults service should put the robot into either 
# DISABLED or ENABLED.
uint8 FAULTED = 2
# Connected to the robot, sending cyclic commands and receiving cyclic
# status values.
uint8 ENABLED = 3

# The current state of the robot driver.
# Note that this can diverge from the state of IconHwmController.
# A robot driver might automatically move between all three states,
# but IconHwmController requires the user to trigger transitions
# between them.
#
# In practice, this means that an OperationalStatus topic can 
# * cause IconHwmController to transition to FAULTED
# * prevent IconHwmController from transitioning away from DISABLED
#
# but never cause IconHwmController to transition to 
# DISABLED or ENABLED.
uint8 state
# Error message, only populated if state == FAULTED.
string message
