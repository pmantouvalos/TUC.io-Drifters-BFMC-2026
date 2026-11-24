import threading

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadOakD(ThreadWithStop):
    """This thread handles OakD.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadOakD, self).__init__()
        
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging      

    def state_change_handler(self):
        pass

    def thread_work(self):
        pass




