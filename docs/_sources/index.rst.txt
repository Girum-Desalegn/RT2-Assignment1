.. assignment_2_2023 documentation master file, created by
   sphinx-quickstart on Mon May  6 21:42:52 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Assignment_2 The two-wheelded mobile robot documentation!
=========================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

The two-wheelded mobile robot documentation!
********************************************
This documentation outlines the motion behavior of a two-wheeled mobile robot within a predefined area, subject to constraints imposed by a blocking task, which restricts movement until the robot reaches a specified target. To achieve this, task implementations rely on an action server.

The two-wheeled mobile robot maneuvers through a three-dimensional space, navigating around obstacles to attain a predefined position. The robot operates within the Gazebo simulation environment, offering user-friendly interaction capabilities.

For controlling the robot's locomotion in this simulated environment, an action server is deployed, utilizing the bug0 algorithm. This documentation has been explained action_client_Node_A.py,service_return_Node_B.py, and service_subscriber_Node_C.py codes to resolve the given problems.

Action_client Module
===========================
.. automodule:: scripts.action_client_Node_A
   :imported-members:
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Service_return Module
===========================
.. automodule:: scripts.service_return_Node_B
   :imported-members:
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Service_subscriber Module
===========================
.. automodule:: scripts.service_subscriber_Node_C
   :imported-members:
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:
