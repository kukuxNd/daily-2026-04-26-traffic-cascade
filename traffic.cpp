#include <SDL2/SDL.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <random>

constexpr int W = 900, H = 400;
constexpr int ROAD_Y1 = 120, ROAD_Y2 = 280;
constexpr int NUM_CARS = 80;
constexpr float ROAD_LEN = 1000.0f;
constexpr float A_MAX = 2.0f;
constexpr float B_MAX = 4.0f;
constexpr float V0 = 33.33f;
constexpr float S0 = 2.0f;
constexpr float T = 1.5f;
constexpr float LAMBDA = 0.5f;

struct Car {
    float x;
    float v;
    float a;
    float color_h;
};

std::vector<Car> cars(NUM_CARS);
std::mt19937 rng;

void init() {
    rng.seed(42);
    float density = 35.0f;
    float spacing = ROAD_LEN / NUM_CARS;
    for (int i = 0; i < NUM_CARS; i++) {
        cars[i].x = i * spacing;
        cars[i].v = V0 * (0.85f + 0.15f * (i % 3) / 3.0f);
        cars[i].a = 0;
        cars[i].color_h = 0.55f + 0.1f * (i % 5) / 5.0f;
    }
}

float s_star(const Car& ego, const Car& lead) {
    float dx = lead.x - ego.x - 4.5f;
    if (dx < 0) dx += ROAD_LEN;
    float dv = ego.v - lead.v;
    return S0 + std::max(0.0f, ego.v * T + ego.v * dv / (2.0f * std::sqrt(A_MAX * B_MAX)));
}

float accel(const Car& ego, const Car& lead) {
    float s = lead.x - ego.x - 4.5f;
    if (s < 0) s += ROAD_LEN;
    float s_star_val = s_star(ego, lead);
    float a_ego = A_MAX * (1.0f - std::pow(ego.v / V0, 4.0f) - std::pow(s_star_val / std::max(s, 0.1f), 2.0f));
    return std::max(-B_MAX, std::min(A_MAX, a_ego));
}

void perturb() {
    int idx = rng() % NUM_CARS;
    cars[idx].v *= (0.6f + 0.1f * (rng() % 5));
}

void update(float dt) {
    for (int iter = 0; iter < 4; iter++) {
        std::vector<float> a(NUM_CARS);
        for (int i = 0; i < NUM_CARS; i++) {
            int lead = (i + 1) % NUM_CARS;
            a[i] = accel(cars[i], cars[lead]);
        }
        for (int i = 0; i < NUM_CARS; i++) {
            cars[i].v += a[i] * dt;
            cars[i].v = std::max(0.0f, cars[i].v);
            cars[i].x += cars[i].v * dt;
            if (cars[i].x >= ROAD_LEN) cars[i].x -= ROAD_LEN;
        }
    }
}

SDL_Color hsv2rgb(float h, float s, float v) {
    int hi = int(h * 6);
    float f = h * 6 - hi;
    float p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
    float r, g, b;
    switch (hi % 6) {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        case 5: r=v; g=p; b=q; break;
    }
    return {Uint8(r*255), Uint8(g*255), Uint8(b*255), 255};
}

void draw(SDL_Renderer* r, int frame) {
    SDL_SetRenderDrawColor(r, 10, 8, 20, 255);
    SDL_RenderClear(r);

    SDL_SetRenderDrawColor(r, 30, 30, 45, 255);
    SDL_Rect road = {50, ROAD_Y1, W-100, ROAD_Y2-ROAD_Y1};
    SDL_RenderFillRect(r, &road);

    for (int lane = 0; lane < 2; lane++) {
        int y = ROAD_Y1 + 20 + lane * 70;
        SDL_SetRenderDrawColor(r, 60, 60, 80, 255);
        for (float x = 50; x < W-50; x += 30) {
            SDL_Rect dash = {int(x), y, 18, 3};
            SDL_RenderFillRect(r, &dash);
        }
    }

    for (int i = 0; i < NUM_CARS; i++) {
        float rx = 50.0f + (cars[i].x / ROAD_LEN) * (W - 100);
        int lane = (i % 2);
        float ry = ROAD_Y1 + 40 + lane * 70;

        float speed_ratio = cars[i].v / V0;
        float h = 0.55f + (1.0f - speed_ratio) * 0.35f;
        SDL_Color col = hsv2rgb(h, 0.9f, 0.9f);

        int car_w = lane == 0 ? 14 : 18;
        SDL_Rect car_rect = {int(rx) - car_w/2, int(ry) - 6, car_w, 12};
        SDL_SetRenderDrawColor(r, col.r, col.g, col.b, 255);
        SDL_RenderFillRect(r, &car_rect);

        SDL_Rect headlight = {int(rx) + car_w/2 - 2, int(ry) - 3, 3, 6};
        SDL_SetRenderDrawColor(r, 255, 255, 200, 200);
        SDL_RenderFillRect(r, &headlight);
    }

    SDL_SetRenderDrawColor(r, 80, 80, 100, 255);
    SDL_Rect info_bg = {10, H-60, 200, 50};
    SDL_RenderFillRect(r, &info_bg);
    SDL_Color white = {200, 200, 220, 255};
    SDL_Color dark  = {20, 20, 35, 255};

    auto print = [&](int x, int y, const char* txt) {
        // lightweight: skip font, info only
    };
    (void)print;

    for (int i = 0; i < NUM_CARS; i++) {
        int lead = (i + 1) % NUM_CARS;
        float s = lead > i ? cars[lead].x - cars[i].x - 4.5f : ROAD_LEN - cars[i].x + cars[lead].x - 4.5f;
        float sf = s_star(cars[i], cars[lead]);
        if (s < sf * 0.5f) {
            float rx = 50.0f + (cars[i].x / ROAD_LEN) * (W - 100);
            int lane = (i % 2);
            float ry = ROAD_Y1 + 40 + lane * 70;
            SDL_Rect warn = {int(rx)-8, int(ry)-10, 16, 20};
            SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_ADD);
            SDL_SetRenderDrawColor(r, 255, 80, 80, 100);
            SDL_RenderFillRect(r, &warn);
            SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);
            break;
        }
    }

    SDL_RenderPresent(r);
    (void)frame;
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Traffic Cascade — Butterfly Effect", 100, 100, W, H, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    init();
    int frame = 0;
    bool quit = false;
    bool auto_perturb = true;
    int perturb_frame = 180;

    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) perturb();
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) init();
        }

        if (auto_perturb && frame == perturb_frame) {
            perturb();
            auto_perturb = false;
        }

        update(0.25f);
        draw(ren, frame++);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
