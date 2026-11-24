if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.localization.Localizer.threads.threadLocalizer import threadLocalizer
from src.pathplanning.Navigator.threads.threadNavigator import threadNavigator


class processLocalizer(WorkerProcess):
    """This process handles Localizer.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processLocalizer, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create the Localizer Publisher thread and add to the list of threads."""
        LocalizerTh = threadLocalizer(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(LocalizerTh)

        NavigatorTh = threadNavigator(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(NavigatorTh)
