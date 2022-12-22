"""Wrapper around `ipp.Client`, `ipp_routines` and `v2routines` to give access to the CMM through G code"""
import ipp
import asyncio
import ipp_routines as routines
import v2routines
from functools import partial

import logging

logger = logging.getLogger(__name__)

cmmInstance = None

def do_nothing(*args, **kwargs): 
  """
  Is called when skip_cmm is True and a cmm, cmm.v2routines or cmm.routines method is called. Does
  nothing and returns None.
  """
  pass

class V2RoutinesProxy:
  """
  Proxy object that calls functions in `v2routines`, automatically passing in the `ipp.Client`
  object and silently skipping the calls if skip_cmm is set on the containing cmm object.
  """
  def __init__(self):
    self.skip_cmm = False
    self.client = None

  def __getattr__(self, name):
    logger.debug("in __getattr__ %s", name)
    if self.skip_cmm:
      logger.debug("Skipped call to cmm.routines.%s", name)
      return do_nothing

    return partial(getattr(v2routines,name), self.client)

class RoutinesProxy:
  """
  Proxy object that calls functions in `ipp_routines`, automatically passing in the `ipp.Client`
  object and silently skipping the calls if skip_cmm is set on the containing cmm object.
  """
  def __init__(self):
    self.skip_cmm = False
    self.client = None

  def __getattr__(self, name):
    logger.debug("in __getattr__ %s", name)
    if self.skip_cmm:
      logger.debug("Skipped call to cmm.routines.%s", name)
      return do_nothing

    return partial(getattr(routines,name), self.client)

class Cmm:
  """
  Class that wraps `ipp.Client` `ipp_routines` and `v2routines` for easy interactions with the CMM.
  Meant to be used as a singleton object that can be used to send CMM commands from an asyncio context.
  These are helpful in an oword context to define CMM behavior that can be called from G code, or can
  be used in standalone python scripts. 

  Example:

  ```
  from cmmmanager import Cmm
  from ipp import Csy

  async def do_some_stuff():
    cmm = Cmm.getInstance()

    await cmm.GoTo("X(%s),Y(%s),Z(%s)" % (100,200,300)).complete()
    await cmm.v2routines.go_to_clearance_y()
    await cmm.routines.set_part_csy(Csy(100,200,300,0,-90,0))
  ```

  """

  def getInstance():
    """
    Returns the global singleton Cmm object.
    """
    global cmmInstance

    if cmmInstance == None:
      cmmInstance = Cmm()

    return cmmInstance

  def __init__(self):
    self.client = None
    self.skip_cmm = False
    self.routines = RoutinesProxy()
    self.v2routines = V2RoutinesProxy()

  def set_skip_cmm(self, value):
    """
    Sets the skip_cmm parameter. When set to true, any calls to the cmm object will do nothing
    and return None. This can be helpful to debug behaviors in an oword that don't require the
    CMM, but have CMM movements that take time you don't want to wait for.
    """
    self.skip_cmm = value
    self.routines.skip_cmm = value
    self.v2routines.skip_cmm = value

  def __getattr__(self, name):
    logger.debug("in __getattr__ %s", name)
    if self.skip_cmm:
      logger.debug("Skipped call to cmm.%s", name)
      return do_nothing

    if not self.client:
      self.client = ipp.Client()
      self.routines.client = self.client
      self.v2routines.client = self.client

    return getattr(self.client, name)
