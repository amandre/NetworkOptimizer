# NetworkOptimizer
Script optimizing the traffic for network topology imported from XML file

It calculates the shortest paths, provides the capacity for the edges on the path.
Traffic is divided between paths based on predefined limitations:
- maximum available capacity per module (in Gbps)
- maximum amount of modules that are possible to install on edge
