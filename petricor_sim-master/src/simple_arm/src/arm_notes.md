def ik_lm_chan(
        self,
        Tep: Union[ndarray, SE3],
        q0: Union[ndarray, None] = None,
        ilimit: int = 30,
        slimit: int = 100,
        tol: float = 1e-6,
        reject_jl: bool = True,
        we: Union[ndarray, None] = None,
        λ: float = 1.0,
    ) -> Tuple[ndarray, int, int, int, float]:
        """
        Numerical inverse kinematics by Levenberg-Marquadt optimization (Chan's Method)

        :param Tep: The desired end-effector pose or pose trajectory
        :param q0: initial joint configuration (default to random valid joint
            configuration contrained by the joint limits of the robot)
        :param ilimit: maximum number of iterations per search
        :param slimit: maximum number of search attempts
        :param tol: final error tolerance
        :param reject_jl: constrain the solution to being within the joint limits of
            the robot (reject solution with invalid joint configurations and perfrom
            another search up to the slimit)
        :param we: a mask vector which weights the end-effector error priority.
            Corresponds to translation in X, Y and Z and rotation about X, Y and Z
            respectively
        :param λ: value of lambda for the damping matrix Wn

        :return: inverse kinematic solution
        :rtype: tuple (q, success, iterations, searches, residual)

``sol = ets.ik_lm_chan(Tep)`` are the joint coordinates (n) corresponding
        to the robot end-effector pose ``Tep`` which is an ``SE3`` or ``ndarray`` object.
        This method can be used for robots with any number of degrees of freedom.
        The return value ``sol`` is a tuple with elements:

        ============    ==========  ===============================================
        Element         Type        Description
        ============    ==========  ===============================================
        ``q``           ndarray(n)  joint coordinates in units of radians or metres
        ``success``     int         whether a solution was found
        ``iterations``  int         total number of iterations
        ``searches``    int         total number of searches
        ``residual``    float       final value of cost function
        ============    ==========  ===============================================

        If ``success == 0`` the ``q`` values will be valid numbers, but the
        solution will be in error.  The amount of error is indicated by
        the ``residual``.