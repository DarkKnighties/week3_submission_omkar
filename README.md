# week3_submission_omkar

## Question 1
### Files included: q1_hello_world_talker.py  q1_hello_world_listener.py
#### q1_hello_world_talker.py
Creates a publisher for the topic named '/new'.

Publishes the message "Hello World !" at the frequency to 15Hz to '/new'.

Prints/logs a confirmation message everytime it publishes.
#### q1_hello_world_listener.py
Subcribes to the topic names '/new'.

Logs the content of the message to the console (in this case, "Hello World !").

## Question 2
### Files included: q2_s1_tlker.py  q2_s2_talker.py
#### q2_s1_talker.py
This is the publisher script 1 for the red light green light question.

It publishes a message to the `/s1` topic, which alternates between 'green' and 'red'.

The state changes to 'red' after 10 seconds of being 'green' and vice versa.
#### q2_s2_talker.py
This is the publisher script 2 for the red light green light question.

It subscribes to the `/s1` topic and publishes to the `/s2` topic.

The message alternates between 'green' and 'red' based on the received message.

If it receives 'green', it publishes 'red', and vice versa.
