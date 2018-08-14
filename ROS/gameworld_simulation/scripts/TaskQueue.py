#!/usr/bin/env python

"""
Queue class used to hold the tasks queue for each agent.
The queue class allows for tasks to be prioritised, removed and
added. I may add some more functionality as I progress, most
functions are self-explanatory by their name.
"""

class TaskQueue():
    def __init__(self, max_size=0):
        self.queue = []
        if max_size != 0:
            self.max_size = max_size
        else:
            self.max_size = None

    def __str__(self):
        return ", ".join(self.queue)

    def enqueue(self, data):
        if self.max_size:
            if self.get_size() > self.max_size:
                print("Queue is at maximum size, cannot add anymore tasks until it is dequeued!")
                return False

        self.queue.append(data)
        return True

    def get_size(self):
        return len(self.queue)

    def peek(self):
        if self.get_size() > 0:
            return self.queue[0]
        print("Queue is EMPTY, no tasks in queue!")
        return None

    def dequeue(self):
        if self.get_size() > 0:
            return self.queue.pop(0)
        print("Queue is EMPTY, no tasks in queue!")
        return None

    def get_queue(self):
        return self.queue

    def empty_queue(self):
        self.queue = []
        return True

    def is_empty(self):
        return (len(self.queue) == 0)

    def prioritise(self, data):
        if data in self.queue:
            self.queue.remove(data)
        if self.max_size:
            if self.get_size() > self.max_size:
                print("Queue is at maximum size, popping last task to make space!")
                self.queue.pop()

        self.queue.insert(0, data)
        return True

    def remove(self, data):
        self.queue.remove(data)


