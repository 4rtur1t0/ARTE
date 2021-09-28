
function test_quaternion2aa()

Q = [0.707 0 0.707 0]
[th, u] = quaternion2aa(Q)


Q = [0.7071 -0.7071 0 0]
[th, u] = quaternion2aa(Q)


Q = [1 0 0 0]
[th, u] = quaternion2aa(Q)

Q = [0.5 0.5 0.5 0.5]
[th, u] = quaternion2aa(Q)