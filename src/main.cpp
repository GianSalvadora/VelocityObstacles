#include "raylib.h"
#include <vector>
#include <cmath>
#include <string>

const int W = 900, H = 600;

struct Agent {
    Vector2 pos, vel, prefVel, goal;
    float   radius;
    Color   color;
    bool    reached;
};

Vector2 Add(Vector2 a, Vector2 b)  { return { a.x+b.x, a.y+b.y }; }
Vector2 Sub(Vector2 a, Vector2 b)  { return { a.x-b.x, a.y-b.y }; }
Vector2 Mul(Vector2 a, float s)    { return { a.x*s,   a.y*s   }; }
float   Dot(Vector2 a, Vector2 b)  { return a.x*b.x + a.y*b.y;   }
float   Len(Vector2 a)             { return sqrtf(Dot(a,a));       }
float   Dist(Vector2 a, Vector2 b) { return Len(Sub(a,b));         }
Vector2 Norm(Vector2 a)            { float l=Len(a); return l>0.001f ? Mul(a,1/l) : Vector2{0,0}; }
Vector2 Rotate(Vector2 v, float a) { return { v.x*cosf(a)-v.y*sinf(a), v.x*sinf(a)+v.y*cosf(a) }; }

Vector2 Limit(Vector2 v, float maxL) {
    if (Len(v) > maxL) return Mul(Norm(v), maxL);
    return v;
}

bool InsideVO(const Agent& A, const Agent& B, Vector2 cand) {
    float combined = A.radius + B.radius + 4.0f;
    Vector2 relPos = Sub(B.pos, A.pos);
    float dist = Len(relPos);
    if (dist < combined) return true;

    float halfAngle = asinf(combined / dist);
    Vector2 dir = Norm(relPos);
    Vector2 relVel = Sub(cand, B.vel);
    float relVelLen = Len(relVel);
    if (relVelLen < 0.0001f) return false;

    float cosAngle = fmaxf(-1.0f, fminf(1.0f, Dot(Norm(relVel), dir)));
    return acosf(cosAngle) < halfAngle;
}

Vector2 ComputeSafeVel(const Agent& A, const std::vector<Agent>& agents, int idx, float maxSpeed) {
    bool prefSafe = true;
    for (int i = 0; i < (int)agents.size(); i++) {
        if (i == idx) continue;
        if (InsideVO(A, agents[i], A.prefVel)) { prefSafe = false; break; }
    }
    if (prefSafe) return A.prefVel;

    Vector2 best = { 0, 0 };
    float bestScore = -1e9f;

    for (int s = 0; s < 32; s++) {
        float angle = (float)s / 32 * 2.0f * PI;
        for (float sp : { 1.0f, 0.75f, 0.5f, 0.25f }) {
            Vector2 cand = Mul(Vector2{ cosf(angle), sinf(angle) }, maxSpeed * sp);
            bool safe = true;
            for (int i = 0; i < (int)agents.size(); i++) {
                if (i == idx) continue;
                if (InsideVO(A, agents[i], cand)) { safe = false; break; }
            }
            if (safe) {
                float score = -Len(Sub(cand, A.prefVel));
                if (score > bestScore) { bestScore = score; best = cand; }
            }
        }
    }
    return best;
}

void DrawVOCone(const Agent& A, const Agent& B) {
    float combined = A.radius + B.radius + 4.0f;
    Vector2 relPos = Sub(B.pos, A.pos);
    float dist = Len(relPos);
    if (dist < 0.01f) return;

    float halfAngle = asinf(fminf(combined / dist, 0.999f));
    Vector2 dir = Norm(relPos);
    float coneLen = 80.0f;

    DrawLineV(A.pos, Add(A.pos, Mul(Rotate(dir, -halfAngle), coneLen)), { 255, 80, 80, 60 });
    DrawLineV(A.pos, Add(A.pos, Mul(Rotate(dir,  halfAngle), coneLen)), { 255, 80, 80, 60 });
}

int main() {
    InitWindow(W, H, "Velocity Obstacles");
    SetTargetFPS(60);

    std::vector<Agent> agents;

    auto MakeAgent = [&](Vector2 pos, Vector2 goal, Color col) {
        agents.push_back({ pos, {0,0}, {0,0}, goal, 14.0f, col, false });
    };

    auto Reset = [&]() {
        agents.clear();
        MakeAgent({ 100, 300 }, { 800, 300 }, { 80,  180, 255, 255 });
        MakeAgent({ 800, 300 }, { 100, 300 }, { 255, 120,  80, 255 });
        MakeAgent({ 100, 150 }, { 800, 450 }, { 100, 220, 100, 255 });
        MakeAgent({ 800, 450 }, { 100, 150 }, { 220, 100, 220, 255 });

        int N = 6; float cx = 450, cy = 300, r = 180;
        for (int i = 0; i < N; i++) {
            float a = i * 2.0f * PI / N;
            Color col = { (unsigned char)(100+rand()%155), (unsigned char)(100+rand()%155), (unsigned char)(100+rand()%155), 255 };
            MakeAgent({ cx+cosf(a)*r, cy+sinf(a)*r }, { cx+cosf(a+PI)*r, cy+sinf(a+PI)*r }, col);
        }
    };

    Reset();

    bool showVO = true, showGoals = true, showVel = true;
    float maxSpeed = 2.5f;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) Reset();
        if (IsKeyPressed(KEY_V)) showVO    = !showVO;
        if (IsKeyPressed(KEY_G)) showGoals = !showGoals;
        if (IsKeyPressed(KEY_A)) showVel   = !showVel;

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mp   = GetMousePosition();
            Vector2 goal = { (float)(rand()%(W-100)+50), (float)(rand()%(H-100)+50) };
            Color col    = { (unsigned char)(80+rand()%175), (unsigned char)(80+rand()%175), (unsigned char)(80+rand()%175), 255 };
            agents.push_back({ mp, {0,0}, {0,0}, goal, 14.0f, col, false });
        }

        for (int i = 0; i < (int)agents.size(); i++) {
            Agent& a = agents[i];
            if (a.reached) continue;

            float distToGoal = Dist(a.pos, a.goal);
            if (distToGoal < a.radius + 2.0f) { a.reached = true; a.vel = {0,0}; continue; }

            float speed = fminf(maxSpeed, distToGoal * 0.05f + 1.0f);
            a.prefVel   = Mul(Norm(Sub(a.goal, a.pos)), speed);
            a.vel       = Limit(ComputeSafeVel(a, agents, i, maxSpeed), maxSpeed);
            a.pos       = Add(a.pos, a.vel);
            a.pos.x     = fmaxf(a.radius, fminf(W-a.radius, a.pos.x));
            a.pos.y     = fmaxf(a.radius, fminf(H-a.radius, a.pos.y));
        }

        BeginDrawing();
        ClearBackground({ 12, 12, 20, 255 });

        if (showVO)
            for (int i = 0; i < (int)agents.size(); i++)
                for (int j = 0; j < (int)agents.size(); j++)
                    if (i != j) DrawVOCone(agents[i], agents[j]);

        if (showGoals)
            for (auto& a : agents) {
                if (a.reached) continue;
                DrawLineEx(a.pos, a.goal, 1.0f, { a.color.r, a.color.g, a.color.b, 40 });
                DrawCircleV(a.goal, 5, { a.color.r, a.color.g, a.color.b, 120 });
            }

        if (showVel)
            for (auto& a : agents) {
                if (a.reached) continue;
                DrawLineEx(a.pos, Add(a.pos, Mul(a.vel,     12.0f)), 2.0f, YELLOW);
                DrawLineEx(a.pos, Add(a.pos, Mul(a.prefVel, 12.0f)), 1.0f, { 255,255,255,80 });
            }

        for (auto& a : agents) {
            if (a.reached) {
                DrawCircleV(a.pos, a.radius, { a.color.r, a.color.g, a.color.b, 80 });
                DrawCircleLinesV(a.pos, a.radius, { a.color.r, a.color.g, a.color.b, 120 });
            } else {
                DrawCircleV(a.pos, a.radius+3, { a.color.r, a.color.g, a.color.b, 30 });
                DrawCircleV(a.pos, a.radius, a.color);
                DrawCircleLinesV(a.pos, a.radius, WHITE);
            }
        }

        DrawRectangle(0, 0, 260, 140, { 0, 0, 0, 160 });
        DrawText("Velocity Obstacles",          10, 10,  18, RAYWHITE);
        DrawText("[R] Reset",                   10, 35,  14, GRAY);
        DrawText("[V] Toggle VO cones",         10, 52,  14, showVO    ? GREEN : GRAY);
        DrawText("[G] Toggle goal lines",       10, 69,  14, showGoals ? GREEN : GRAY);
        DrawText("[A] Toggle velocity arrows",  10, 86,  14, showVel   ? GREEN : GRAY);
        DrawText("[Click] Add agent",           10, 103, 14, GRAY);
        DrawText(("Agents: " + std::to_string(agents.size())).c_str(), 10, 120, 14, RAYWHITE);
        DrawFPS(W-90, 10);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}