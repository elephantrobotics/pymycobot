import sys, time, threading, logging
sys.path.append('.')

from ai import singleton, parameters, functools, coloredlogging

logger = coloredlogging.get_logger("AS", coloredlogging.BLUE)
logger.addHandler(logging.NullHandler())

class AppStates(metaclass=singleton.Singleton):
    happiness = 4
    hungry = 4
    energy = 4
    comfort = 4
    counter = 0

    def __init__(self) -> None:
        ...

    @classmethod
    def recover(cls, genre):
        if genre == 'energy':
            cls.energy += 1
            cls.energy = min(4, cls.energy)
        elif genre == 'happiness':
            cls.happiness += 1
            cls.happiness = min(4, cls.happiness)
        elif genre == 'hungry':
            cls.happiness += 1
            cls.happiness = min(4, cls.happiness)
        elif genre == 'comfort':
            cls.comfort += 1
            cls.comfort = min(4, cls.comfort)

    @classmethod
    def drop(cls, genre: str):
        if genre == 'energy':
            cls.energy -= 1
            cls.energy = max(0, cls.energy)
        elif genre == 'happiness':
            cls.happiness -= 1
            cls.happiness = max(0, cls.happiness)
        elif genre == 'hungry':
            cls.happiness -= 1
            cls.happiness = max(0, cls.happiness)
        elif genre == 'comfort':
            cls.comfort -= 1
            cls.comfort = max(0, cls.comfort)

    @classmethod
    def recover_by_action(cls, action: str):
        if action == 'eat':
            cls.recover('hungry')
        elif action.startswith('touch'):
            cls.recover('comfort')
        elif action in parameters.BEHAVIOURS['relax']:
            cls.recover('energy')
        elif action.startswith('voice') or action.startswith(
                'stare') or action in ['ball', 'attack', 'flap']:
            cls.recover('happiness')

    @classmethod
    @functools.thread_run
    def async_run(cls):
        logger.debug(f'Starting Stats recorder')
        while ...:
            time.sleep(60 * 10)
            AppStates.counter += 1
            cls.drop('energy')
            if AppStates.counter % 2 == 0:
                cls.drop('happiness')
                cls.drop('comfort')
            elif AppStates.counter % 12 == 0:
                cls.drop('hungry')

