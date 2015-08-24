import re
import time
import datetime
from collections import OrderedDict

LOG = "../share/interaction.log"

# We code here what is the expected focus of attention of the child for each task/event.
# This is a simplified model: in many situations, several different focuses may be expected
# (possibly with associated probabilities).
log2frame = {"WAITING_FOR_WORD": "selection_tablet",
             "WORD_RECEIVED": "robot_head",
             "FEEDBACK_ON_WORD": "robot_head",
             "WRITING": "tablet",
             "PROMPTING_FOR_DEMONSTRATION": "robot_head",
             "WAITING_FOR_DEMONSTRATION": "tablet",
             "DEMONSTRATION_RECEIVED": "robot_head",
             "FEEDBACK_ON_DEMONSTRATION": "robot_head"
             }

action = re.compile('(?P<time>[\d :-]*) - interaction_manager.*(?P<type>' + \
                    "|".join(log2frame.keys()) + \
                    ").*")

def read_log(rawlog):
    """ read and pre-process the log
    """
    first_time = None
    log = OrderedDict()
    with open(rawlog, 'r') as logfile:
        for line in logfile.readlines():
            found = action.search(line)
            if found:
                evt_time = datetime.datetime.strptime(found.group('time'), "%Y-%m-%d %H:%M:%S")
                expected_frame = log2frame[found.group('type')]

                if first_time is None:
                    first_time = evt_time
                    log[0] = expected_frame
                else:
                    secs_from_start = (evt_time - first_time).total_seconds()
                    log[secs_from_start] = expected_frame
            else:
                print("Malformed log file!")

    return log

def get_current_expected_frame(log, start_time):
    """ Returns the currently expected frame based on the log and
        a start point in time.
    """
    current_expected_frame = None
    elapsed_time = (datetime.datetime.now() - start_time).total_seconds()

    for log_times in log.keys():
        if elapsed_time > log_times:
            current_expected_frame = log[log_times]
        else:
            break

    return current_expected_frame


if __name__ == "__main__":

    log = read_log(LOG)


    start_time = datetime.datetime.now()
    expected = None
    last_expected = None
    while True:

        expected = get_current_expected_frame(log, start_time)
        if expected != last_expected:
            elapsed_time = int((datetime.datetime.now() - start_time).total_seconds())
            print("[%s sec after start] Expected frame: %s" % (elapsed_time, expected))

        last_expected = expected

        time.sleep(0.1)
