from BaseRobot import BaseRobot, BaseBody, BaseNetwork
from MockBody import MockBody
from MockNetwork import MockNetwork
from Map import Map2D
import imp

__all__ = ['BaseRobot', 'BaseBody', 'BaseNetwork',
           'MockBody', 'MockNetwork',
           'Map2D']

# Include eBotBody only if eBot-API is installed
try:
    imp.find_module('eBot')
    include_eBot = True
except ImportError:
    include_eBot = False

if include_eBot:
    from eBotBody import eBotBody
    __all__.append('eBotBody')
del include_eBot

# Include XBee*Network only if serial is installed
try:
    imp.find_module('serial')
    include_serial = True
except ImportError:
    include_serial = False

if include_serial:
    from XBeeNetwork import XBeeNetwork, XBeeExpirationNetwork
    __all__.extend(['XBeeNetwork', 'XBeeExpirationNetwork'])
del include_serial
