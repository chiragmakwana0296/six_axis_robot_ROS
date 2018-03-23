#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Chiragkumar Makwana.
# Released under the BSD License.
#
# Author:
#   * Chiragkumar Makwana

import numpy as np


def map_(X,Xmax,Xmin,Ymax,Ymin):
    m = (Ymax-Ymin)/(Xmax-Xmin)
    c=Ymin
    Y=m*X+c
    return Y

if __name__ == '__main__':
    for x in range(0,360,45):
        y=map_(x,360,0,90,-90)
        print(y)
