# Evaluation of a robot's ([Wowbagger](http://www.ist.tugraz.at/ais-wiki/wowbagger)) Detection System
Evaluation and Improvement of the Current Victim Detection System of a Rescue Robot [Wowbagger](http://www.ist.tugraz.at/ais-wiki/wowbagger)

[![Build Status](https://travis-ci.org/tompollard/phd_thesis_markdown.svg?branch=master)](https://travis-ci.org/tompollard/phd_thesis_markdown)  

Source: http://www.ist.tugraz.at/ais-wiki/victimdetection

This project presents an analysis of the current state-of-the-art of the Wowbagger’s
Face Detection, in order to find a better approach for detecting victims,
in our case dolls. Wowbagger is the name of the robot, who will send regularly
to competitions. The focus is mainly on image processing for autonomous robots
operating for detecting victims in crisis area. The thesis discuses the methods that
are more adequate to be applied in a rescue scenario and some parts will be chosen
to implement an optimal doll detection for our robot.
It will be discussed one of the main objectives of rescue missions is to rescue the
disaster victims. Of course, in some situations it is too risky to send human agents
in the unstable environment. Therefore, robotic agents can be essential in such
situations and important collaborators aside human agents.
One crucial requirement in above-mentioned rescue scenarios is to identify victims
from all other objects. A task, that ought not to be difficult for us humans, but
indeed, it is for robots. A rescue team benefits from the abilities of an autonomous
robot, which is able to intervene dangerous areas.
In order to detect an victim, an autonomous robot must must be equipped with a
set of specific sensors, those provide information about the environment.
In this case, we have “Wowbagger”, the autonomous robot of the RoboCup Rescue
Team of Graz University of Technology.

## Installation

A toolchain is provided to start the project out of the box.

  * ROS Indigo
  * C++11 >= 6

## Usage

    make help // shows you a list of options

## History

This file started with LinkerScript Parser v0.0 and tracks the feature of this tool.
Each line will describe a single addition/removement/change and adhere to the following format:

`<Author> <Type> : <Textual_description_without_linebreak>`

Authors (so far):

  * PL   Peter Lorenz

Type:

  * \+ Addition
  * \- Removement
  * \# Modification
  * \~ Fix
  * \! Misc.
  * (very significant entries should be in upper case and prefixed with "-----" )

Table of changes:

  * `PL + The initial setup is done. Version 0.0`
  * todo add changes

## Credits

  * Part of my BSc Project, written by Peter Lorenz.

## License

GNU General Public License v3.0
