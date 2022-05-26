# robontics2022
Scratchpad for code associated with a RobOntics 2022 paper

# Files

src/percsym.py

Functions to convert a "frame", containing numeric data about object poses, movement, and contacts, and convert them into symbolic descriptions. Types of questions:

a) is an object supported? By what? (meaning: is it falling? if not, does it contact another object in a way that exerts a force against gravity?)
b) is an object in a region/what objects are in region?
c) are two objects placed in such a way relative to each other that one of these descriptions applies: in front/behind/above/below/left of/right of? (meaning: given a relevant coordinate system, is the displacement between the objects oriented along the relevant axis?)
d) is object moving? is object moving in a certain way (approach, depart, not at all) relative to another?
e) is robot moving? (meaning: is there motion in some actuators that typically cause the base to move?)
