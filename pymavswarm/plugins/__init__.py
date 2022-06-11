from typing import Callable, List

from .hrl_plugin import HrlPlugin
from .plugin import Plugin

# The list of supported plugins
# Add your plugin to this list to add support for your new plugin
# If there is a plugin that isn't used, the plugin may be commented out
plugins = [HrlPlugin]
