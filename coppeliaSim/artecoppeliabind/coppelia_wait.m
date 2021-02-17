function coppelia_wait(coppelia, n)
    for i=1:n
        coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
    end
end