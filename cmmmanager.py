# Wrapper around ipp.Client to give access to the CMM through G code
import ipp
import asyncio
import ipp_routines as routines
import v2routines
from functools import partial

import logging

logger = logging.getLogger(__name__)

cmmInstance = None

def do_nothing(*args, **kwargs): pass

class V2RoutinesProxy:
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
  def getInstance():
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
