# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from carebt.abstractLogger import AbstractLogger
from carebt.abstractLogger import LogLevel


class RosLogger(AbstractLogger):
    """Bridge careBT logging into ROS logging."""

    def __init__(self, logger):
        super().__init__()
        self._logger = logger

    def trace(self, msg: str):
        if self._log_level <= LogLevel.TRACE:
            self._logger.debug(msg)

    def debug(self, msg: str):
        if self._log_level <= LogLevel.DEBUG:
            self._logger.debug(msg)

    def info(self, msg: str):
        if self._log_level <= LogLevel.INFO:
            self._logger.info(msg)

    def warn(self, msg: str):
        if self._log_level <= LogLevel.WARN:
            self._logger.warning(msg)

    def error(self, msg: str):
        if self._log_level <= LogLevel.ERROR:
            self._logger.error(msg)
