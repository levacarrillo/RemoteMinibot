typedef struct movement_ {
    float twist;
    float advance;
} movement;

enum Behaviors {
    NONE,
    LIGHT_FOLLOWER,
    SM_DESTINATION,
    SM_AVOID_OBSTACLES,
    SM_AVOIDANCE_DESTINATION,
    SM_ORACLE_CLIPS,
    DFS,
    DIJKSTRA,
    USER_SM,
    ACTION_PLANNER,
    LINE_FOLLOWER,
    NOT_DEFINED
};