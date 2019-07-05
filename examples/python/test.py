#!/usr/bin/env python

import centroidal_planner.pycpl as cpl
import numpy as np


def test_centroidal_planner():
    env = cpl.Ground()
    env.SetGroundZ(0.1)

    contacts = ['micio', 'miao']
    mass = 20.0

    planner = cpl.CentroidalPlanner(contacts, mass, env)

    sol = planner.Solve()

    print sol


def test_com_planner():

    contacts = ['c_1', 'c_2', 'c_3', 'c_4']
    mass = 100.0
    mu = 0.5
    com_ref = [0.2, 0.2, 1.0]

    com_pl = cpl.CoMPlanner(contacts, mass)
    com_pl.SetMu(mu)

    com_pl.SetCoMRef(com_ref)
    com_pl.SetPosition('c_1', [1.0, 1.0, 0.0])
    com_pl.SetPosition('c_2', [-1.0, 1.0, 0.0])
    com_pl.SetPosition('c_3', [1.0, -1.0, 0.0])
    com_pl.SetPosition('c_4', [-1.0, -1.0, 0.0])

    com_pl.SetLiftingContact('c_3')
    com_pl.SetLiftingContact('c_4')

    sol = com_pl.Solve()

    print(sol)


def main():

    test_centroidal_planner()
    test_com_planner()


if __name__ == '__main__':
    main()