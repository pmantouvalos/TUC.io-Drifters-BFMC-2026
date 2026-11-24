if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.control.controller.threads.threadcontroller import threadcontroller

class processcontroller(WorkerProcess):
    """This process handles controller.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self,sharedMemory, queueList, logging, ready_event=None , debugging=False): # remove sharedmemeory if kaboom
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.sharedMemory = sharedMemory # remove if sucks
        super(processcontroller, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create the controller Publisher thread and add to the list of threads."""
        controllerTh = threadcontroller(
            self.sharedMemory,self.queuesList, self.logging, self.debugging
        )
        self.threads.append(controllerTh)  # REMOVE SHARED MEMORY IF SUCKS
