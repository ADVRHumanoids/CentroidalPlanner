#!/usr/bin/env python

import centroidal_planner.pycpl as cpl
import numpy as np


def main():

    env = cpl.Ground()
    env.SetGroundZ(0.1)

    contacts = ['micio', 'miao']
    mass = 20.0

    planner = cpl.CentroidalPlanner(contacts, mass, env)

    sol = planner.Solve()

    print sol


if __name__ == '__main__':
    main()