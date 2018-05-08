#!/usr/bin/env python

from pi_trees_ros.pi_trees_ros import *
import time


class ParallelExample:

    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")

        # The message to print
        message = "Take me to your leader!"

        # How high the counting task should count
        n_count = 10

        # Create a PrintMessage() task as defined later fin the script
        PRINT_MESSAGE = PrintMessage("PRINT_MESSAGE", message)


class PrintMessage(Task):

    def __init__(self, name, message, *args, **kwargs):
        super(PrintMessage, self).__init__(name, *args, **kwargs)

        self.name = name
        self.message = message
        self.words = message.split()

        print "Creating Print Message task for", self.message

    def run(self):
        try:
            word = self.words.pop(0)
            print word
            time.sleep(0.1)
            if self.words == []:
                return TaskStatus.SUCCESS
            return TaskStatus.RUNNING
        except:
            return TaskStatus.SUCCESS

    def reset(self):
        self.words = self.message.split()


class Count(Task):

    def __init__(self, name, number, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)

        self.name = name
        self.number = number
        self.count = 0

        print ""


if __name__ == '__main__':
    try:
        tree = ParallelExample()
    except rospy.ROSInterruptException:
        pass
