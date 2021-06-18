# -*- coding: utf-8 -*-

"""
This file contains the Qudi GUI module utility classes.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import sys, inspect
# Do not remove the two lines below! Those two modules are used implicitly.
from gui.colordefs import *
from gui.mycolordefs import *

class ListColorScale:
    """ Make a list of color scales defined in this module and all loaded sub-modules.
    The list is formed from the names of classes whose name begins with `ColorScale`.

    This class is used to make a ComboBox with a list of available colormaps in GUI.
    """
    # Get a list of all classes in this module - returns a list of tuples
    class_list = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    # Convert list to dictionary to call the needed class by its name from GUI
    cs_dict = {}
    for item in class_list:
        cs_dict[item[0]] = item[1]
    # Get a list of class names only - drop everything else
    class_name_list = [item[0] for item in class_list]
    # Get a list of only those class names that starts with 'ColorScale' (they define color scales)
    cs_classes = [name for name in class_name_list if name.startswith('ColorScale') and name != 'ColorScale']
    # Drop the 'ColorScale' prefix in the name of color scales
    cs_names = [name[10:] for name in cs_classes]
