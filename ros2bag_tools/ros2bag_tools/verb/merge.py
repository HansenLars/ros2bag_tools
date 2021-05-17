# Copyright 2021 AIT Austrian Institute of Technology GmbH
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

from ros2bag_tools.filter import BagMessageFilter
from ros2bag_tools.verb import BaseProcessVerb


class MergeVerb(BaseProcessVerb):
    """Merge multiple bags into new bag."""

    def __init__(self):
        # Instantiate identity filter to just pass all topics and messages through for merging
        BaseProcessVerb.__init__(self, BagMessageFilter())