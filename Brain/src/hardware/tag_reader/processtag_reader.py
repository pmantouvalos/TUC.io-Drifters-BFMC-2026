if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.hardware.tag_reader.threads.threadtag_reader import threadtag_reader

class processtag_reader(WorkerProcess):
    """This process handles tag_reader.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processtag_reader, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create the tag_reader Publisher thread and add to the list of threads."""
        tag_readerTh = threadtag_reader(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(tag_readerTh)
