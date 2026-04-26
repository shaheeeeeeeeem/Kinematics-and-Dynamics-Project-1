#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace sf;

constexpr float PI = 3.14159265359f;
constexpr float dt = 0.001f;

// Renamed from State to MechState to avoid clash with sf::State
struct MechState
{
    float theta2=0, theta3=0;
    float omega2=0, omega3=0;
    float alpha2=0, alpha3=0;
    float slider=0;
    float sliderVel=0;
    float sliderAcc=0;
};

bool solvePosition(float a, float b, MechState& s)
{
    float sin2 = sin(s.theta2);
    float cos2 = cos(s.theta2);
    float term = b*b - a*a*sin2*sin2;
    if(term < 0) return false;
    float root = sqrt(term);
    s.slider = a*cos2 + root;
    float Ax = a*cos2;
    float Ay = a*sin2;
    s.theta3 = atan2(-Ay, s.slider - Ax);
    return true;
}

void solveVelocity(float a, float b, MechState& s)
{
    float sin2 = sin(s.theta2);
    float cos2 = cos(s.theta2);
    float root = sqrt(b*b - a*a*sin2*sin2);
    s.sliderVel =
        -a*s.omega2*sin2
        - (a*a*s.omega2*sin2*cos2)/root;
    float Ax = a*cos2;
    float Ay = a*sin2;
    float rBAx = s.slider - Ax;
    float VAy = a*s.omega2*cos2;
    s.omega3 = VAy / rBAx;
}

void solveAcceleration(float a, float b, MechState& s)
{
    float sin2 = sin(s.theta2);
    float cos2 = cos(s.theta2);
    float Ax = a*cos2;
    float Ay = a*sin2;
    float rBAx = s.slider - Ax;
    float rBAy = -Ay;
    float AAx = -a*s.alpha2*sin2 - a*s.omega2*s.omega2*cos2;
    float AAy =  a*s.alpha2*cos2 - a*s.omega2*s.omega2*sin2;
    s.alpha3    = (s.omega3*s.omega3*rBAy - AAy) / rBAx;
    s.sliderAcc = AAx - s.alpha3*rBAy - s.omega3*s.omega3*rBAx;
}

int main()
{
    RenderWindow window(
        VideoMode({1200, 800}),
        "Crank Slider Mechanism");

    ImGui::SFML::Init(window);
    Clock deltaClock;

    Font font("C:/Windows/Fonts/arial.ttf");

    float a = 120;
    float b = 300;

    float theta2_deg   = 45;
    float omega2_input = 2;
    float alpha2_input = 0;

    bool running          = false;
    bool showVelocity     = false;
    bool showAcceleration = false;

    RenderWindow velocityWindow;
    RenderWindow accelerationWindow;

    MechState s;
    int inversion = 1;

    while(window.isOpen())
    {
        while(auto e = window.pollEvent())
        {
            ImGui::SFML::ProcessEvent(window, *e);
            if(e->is<Event::Closed>())
                window.close();
        }

        ImGui::SFML::Update(window, deltaClock.restart());

        if(running)
        {
            s.alpha2  = alpha2_input;
            s.omega2 += s.alpha2 * dt;
            s.theta2 += s.omega2 * dt;
        }
        else
        {
            s.theta2 = theta2_deg * PI / 180.f;
            s.omega2 = omega2_input;
            s.alpha2 = alpha2_input;
        }

        solvePosition(a, b, s);
        solveVelocity(a, b, s);
        solveAcceleration(a, b, s);

        window.clear(Color::Black);

        auto rotVec = [](Vector2f v, float ang) -> Vector2f {
            return { v.x*cosf(ang) - v.y*sinf(ang),
                     v.x*sinf(ang) + v.y*cosf(ang) };
        };

        Vector2f O2_disp, A_disp, B_disp;
        float    guideAngle = 0.f;
        Vector2f guideOrigin;

        switch(inversion)
        {
            default:
            case 1:
                O2_disp     = { 500.f, 430.f };
                A_disp      = O2_disp + Vector2f(  a*cosf(s.theta2), -a*sinf(s.theta2) );
                B_disp      = O2_disp + Vector2f( s.slider, 0.f );
                guideAngle  = 0.f;
                guideOrigin = O2_disp;
                break;

            case 2:
                O2_disp     = { 680.f, 420.f };
                A_disp      = O2_disp + Vector2f( a, 0.f );
                B_disp      = O2_disp + rotVec({ s.slider, 0.f }, s.theta2);
                guideAngle  = s.theta2;
                guideOrigin = O2_disp;
                break;

            case 3:
                A_disp      = { 550.f, 430.f };
                B_disp      = A_disp + Vector2f( b, 0.f );
                O2_disp     = A_disp + rotVec({ -a*cosf(s.theta2), a*sinf(s.theta2) }, s.theta3);
                guideAngle  = s.theta3;
                guideOrigin = B_disp;
                break;

            case 4:
                B_disp      = { 870.f, 430.f };
                O2_disp     = B_disp + Vector2f( -s.slider, 0.f );
                A_disp      = O2_disp + Vector2f( a*cosf(s.theta2), -a*sinf(s.theta2) );
                guideAngle  = 0.f;
                guideOrigin = B_disp;
                break;
        }

        // draw helpers
        auto drawLine = [&](Vector2f p1, Vector2f p2, Color col = Color::White)
        {
            Vertex l[2];
            l[0].position = p1; l[0].color = col;
            l[1].position = p2; l[1].color = col;
            window.draw(l, 2, PrimitiveType::Lines);
        };

        auto drawJoint = [&](Vector2f p, Color col = Color::White)
        {
            CircleShape c(6.f);
            c.setOrigin({6.f, 6.f});
            c.setPosition(p);
            c.setFillColor(col);
            window.draw(c);
        };

        auto drawGroundPin = [&](Vector2f p)
        {
            ConvexShape tri;
            tri.setPointCount(3);
            tri.setPoint(0, {  0.f,  0.f });
            tri.setPoint(1, {-10.f, 16.f });
            tri.setPoint(2, { 10.f, 16.f });
            tri.setPosition(p);
            tri.setFillColor(Color(140, 140, 140));
            window.draw(tri);
            for(int i = 0; i < 5; i++)
            {
                float ox = -8.f + i*4.f;
                Vertex h[2];
                h[0].position = p + Vector2f(ox,      16.f);
                h[1].position = p + Vector2f(ox-5.f,  22.f);
                h[0].color = h[1].color = Color(140, 140, 140);
                window.draw(h, 2, PrimitiveType::Lines);
            }
        };

        const Color groundCol(210, 140, 40);

        // rail
        {
            Vector2f dir(cosf(guideAngle), sinf(guideAngle));
            Color gc = (inversion==1 || inversion==4)
                       ? Color(90,90,90)
                       : Color(50,100,50);
            drawLine(guideOrigin - dir*700.f, guideOrigin + dir*700.f, gc);
        }

        // links
        switch(inversion)
        {
            case 1:
                drawLine(O2_disp, A_disp);
                drawLine(A_disp,  B_disp);
                drawGroundPin(O2_disp);
                break;
            case 2:
                drawLine(O2_disp, A_disp, groundCol);
                drawLine(A_disp,  B_disp);
                drawGroundPin(O2_disp);
                drawGroundPin(A_disp);
                break;
            case 3:
                drawLine(O2_disp, A_disp);
                drawLine(A_disp,  B_disp, groundCol);
                drawGroundPin(A_disp);
                drawGroundPin(B_disp);
                break;
            case 4:
                drawLine(O2_disp, A_disp);
                drawLine(A_disp,  B_disp);
                drawGroundPin(B_disp);
                break;
        }

        drawJoint(O2_disp);
        drawJoint(A_disp);
        drawJoint(B_disp);

        // slider block
        {
            RectangleShape block({40.f, 20.f});
            block.setOrigin({20.f, 10.f});
            block.setPosition(B_disp);
            block.setRotation(degrees(guideAngle * 180.f / PI));
            block.setFillColor(inversion==4 ? Color(80,80,80) : Color(200,200,200));
            window.draw(block);
        }

        // labels
        auto drawLabel = [&](const string& t, Vector2f p)
        {
            Text tx(font);
            tx.setString(t);
            tx.setCharacterSize(14);
            tx.setFillColor(Color::Yellow);
            tx.setPosition(p);
            window.draw(tx);
        };

        drawLabel("O2", O2_disp + Vector2f(-20.f, -20.f));
        drawLabel("A",  A_disp  + Vector2f( 10.f, -10.f));
        drawLabel("B",  B_disp  + Vector2f( 10.f, -10.f));

        // inversion title on screen
        {
            static const char* titles[5] = {"",
                "Inversion 1 - Frame fixed (standard crank-slider)",
                "Inversion 2 - Crank fixed (oscillating cylinder)",
                "Inversion 3 - Coupler fixed",
                "Inversion 4 - Slider fixed (rotary engine)"};
            Text title(font);
            title.setString(titles[inversion]);
            title.setCharacterSize(16);
            title.setFillColor(Color::Cyan);
            title.setPosition({350.f, 15.f});
            window.draw(title);
        }

        // ---- ImGui panel ----
        ImGui::Begin("Crank Slider");

        ImGui::InputFloat("Crank radius (a)", &a);
        ImGui::InputFloat("Connecting rod (b)", &b);
        ImGui::SliderFloat("Theta2", &theta2_deg, 0, 360);
        ImGui::InputFloat("Omega2", &omega2_input);
        ImGui::InputFloat("Alpha2", &alpha2_input);

        if(ImGui::Button(running ? "Stop" : "Start"))
            running = !running;

        if(ImGui::Button("Velocity Diagram"))
        {
            showVelocity = true;
            velocityWindow.create(VideoMode({500,500}), "Velocity Diagram");
        }
        if(ImGui::Button("Acceleration Diagram"))
        {
            showAcceleration = true;
            accelerationWindow.create(VideoMode({500,500}), "Acceleration Diagram");
        }
        if(ImGui::Button("Reset"))
            running = false;

        ImGui::Separator();
        ImGui::Text("Inversions");
        if(ImGui::Button("Inv 1 - Frame fixed (standard)")) inversion = 1;
        if(ImGui::Button("Inv 2 - Crank fixed"))            inversion = 2;
        if(ImGui::Button("Inv 3 - Coupler fixed"))          inversion = 3;
        if(ImGui::Button("Inv 4 - Slider fixed"))           inversion = 4;
        ImGui::Text("Active: Inversion %d", inversion);

        ImGui::Separator();
        ImGui::Text("Position Analysis");
        ImGui::Text("Theta2: %.2f", s.theta2*180/PI);
        ImGui::Text("Theta3: %.2f", s.theta3*180/PI);

        ImGui::Separator();
        ImGui::Text("Velocity Analysis");
        ImGui::Text("Omega2: %.3f", s.omega2);
        ImGui::Text("Omega3: %.3f", s.omega3);
        ImGui::Text("Slider velocity: %.3f", s.sliderVel);

        ImGui::Separator();
        ImGui::Text("Acceleration Analysis");
        ImGui::Text("Alpha2: %.3f", s.alpha2);
        ImGui::Text("Alpha3: %.3f", s.alpha3);
        ImGui::Text("Slider acceleration: %.3f", s.sliderAcc);

        ImGui::End();

        ImGui::SFML::Render(window);
        window.display();

        // ---- Velocity diagram window ----
        if(showVelocity && velocityWindow.isOpen())
        {
            while(auto e = velocityWindow.pollEvent())
                if(e->is<Event::Closed>())
                    velocityWindow.close();

            velocityWindow.clear(Color::Black);

            Vector2f origin(250, 250);

            Vector2f VA(
                -a * s.omega2 * sin(s.theta2),
                 a * s.omega2 * cos(s.theta2)
            );
            Vector2f VBA(
                -b * s.omega3 * sin(s.theta3),
                 b * s.omega3 * cos(s.theta3)
            );
            Vector2f VB(s.sliderVel, 0);

            float Va_mag  = sqrt(VA.x*VA.x   + VA.y*VA.y);
            float Vba_mag = sqrt(VBA.x*VBA.x + VBA.y*VBA.y);
            float Vb_mag  = sqrt(VB.x*VB.x   + VB.y*VB.y);

            float maxMag = max(Va_mag, max(Vba_mag, Vb_mag));
            float scale  = (maxMag > 1e-5f) ? 120.f/maxMag : 1.f;

            Vector2f Va  = VA  * scale;
            Vector2f Vba = VBA * scale;
            Vector2f Vb  = VB  * scale;

            auto drawVec = [&](Vector2f start, Vector2f vec, Color col)
            {
                Vector2f end = start + vec;
                Vertex line[2];
                line[0].position = start; line[0].color = col;
                line[1].position = end;   line[1].color = col;
                velocityWindow.draw(line, 2, PrimitiveType::Lines);
                float size = 10.f, ang = atan2(vec.y, vec.x);
                Vector2f a1(end.x - size*cos(ang-PI/6), end.y - size*sin(ang-PI/6));
                Vector2f a2(end.x - size*cos(ang+PI/6), end.y - size*sin(ang+PI/6));
                Vertex arr[4];
                arr[0].position=end; arr[1].position=a1;
                arr[2].position=end; arr[3].position=a2;
                for(int i=0;i<4;i++) arr[i].color=col;
                velocityWindow.draw(arr, 4, PrimitiveType::Lines);
            };

            drawVec(origin,    Va,  Color::Red);
            drawVec(origin+Vb, Vba, Color::Blue);
            drawVec(origin,    Vb,  Color::White);

            auto vlabel = [&](string t, Vector2f p)
            {
                Text txt(font);
                txt.setString(t); txt.setCharacterSize(14);
                txt.setFillColor(Color::White); txt.setPosition(p);
                velocityWindow.draw(txt);
            };
            vlabel("Va",  (origin+origin+Va)/2.f);
            vlabel("Vba", (origin+Vb+origin+Vb+Vba)/2.f);
            vlabel("Vb",  (origin+origin+Vb)/2.f);

            RectangleShape panel;
            panel.setSize({180,90}); panel.setPosition({10,10});
            panel.setFillColor(Color(30,30,30,200));
            velocityWindow.draw(panel);

            Text txt(font);
            txt.setCharacterSize(14); txt.setFillColor(Color::White);
            txt.setString("Va  = "+to_string(Va_mag)+
                          "\nVba = "+to_string(Vba_mag)+
                          "\nVb  = "+to_string(Vb_mag));
            txt.setPosition({20,20});
            velocityWindow.draw(txt);

            Text scaleTxt(font);
            scaleTxt.setCharacterSize(14); scaleTxt.setFillColor(Color::White);
            scaleTxt.setString("Scale: 1 px = "+to_string(1/scale)+" m/s");
            scaleTxt.setPosition({20,450});
            velocityWindow.draw(scaleTxt);

            Text title(font);
            title.setString("Velocity Diagram");
            title.setCharacterSize(18); title.setPosition({150,20});
            velocityWindow.draw(title);

            velocityWindow.display();
        }

        // ---- Acceleration diagram window ----
        if(showAcceleration && accelerationWindow.isOpen())
        {
            while(auto e = accelerationWindow.pollEvent())
                if(e->is<Event::Closed>())
                    accelerationWindow.close();

            accelerationWindow.clear(Color::Black);

            Vector2f origin(250, 250);

            Vector2f AA(
                -a*s.alpha2*sin(s.theta2) - a*s.omega2*s.omega2*cos(s.theta2),
                 a*s.alpha2*cos(s.theta2) - a*s.omega2*s.omega2*sin(s.theta2)
            );

            // Recompute rBAx/rBAy here (they are local to solveAcceleration)
            float Ax2  = a*cos(s.theta2);
            float Ay2  = a*sin(s.theta2);
            float rBAx = s.slider - Ax2;
            float rBAy = -Ay2;

            Vector2f ABA(
                -s.alpha3*rBAy - s.omega3*s.omega3*rBAx,
                 s.alpha3*rBAx - s.omega3*s.omega3*rBAy
            );
            Vector2f AB(s.sliderAcc, 0);

            float Aa_mag  = sqrt(AA.x*AA.x   + AA.y*AA.y);
            float Aba_mag = sqrt(ABA.x*ABA.x + ABA.y*ABA.y);
            float Ab_mag  = sqrt(AB.x*AB.x   + AB.y*AB.y);

            float maxMag = max(Aa_mag, max(Aba_mag, Ab_mag));
            float scale  = (maxMag > 1e-5f) ? 150.f/maxMag : 1.f;

            Vector2f Aa  = AA  * scale;
            Vector2f Aba = ABA * scale;
            Vector2f Ab  = AB  * scale;

            auto drawVec = [&](Vector2f start, Vector2f vec, Color col)
            {
                Vector2f end = start+vec;
                Vertex line[2];
                line[0].position=start; line[0].color=col;
                line[1].position=end;   line[1].color=col;
                accelerationWindow.draw(line, 2, PrimitiveType::Lines);
                float size=10.f, ang=atan2(vec.y,vec.x);
                Vector2f a1(end.x-size*cos(ang-PI/6), end.y-size*sin(ang-PI/6));
                Vector2f a2(end.x-size*cos(ang+PI/6), end.y-size*sin(ang+PI/6));
                Vertex arr[4];
                arr[0].position=end; arr[1].position=a1;
                arr[2].position=end; arr[3].position=a2;
                for(int i=0;i<4;i++) arr[i].color=col;
                accelerationWindow.draw(arr, 4, PrimitiveType::Lines);
            };

            drawVec(origin,    Aa,  Color::Blue);
            drawVec(origin+Aa, Aba, Color::Green);
            drawVec(origin,    Ab,  Color::Red);

            auto alabel = [&](string t, Vector2f p)
            {
                Text txt(font);
                txt.setString(t); txt.setCharacterSize(14);
                txt.setFillColor(Color::White); txt.setPosition(p);
                accelerationWindow.draw(txt);
            };
            alabel("Aa",  (origin+origin+Aa)/2.f);
            alabel("Aba", (origin+Ab+origin+Ab+Aba)/2.f);
            alabel("Ab",  (origin+origin+Ab)/2.f);

            RectangleShape panel;
            panel.setSize({180,90}); panel.setPosition({10,10});
            panel.setFillColor(Color(30,30,30,200));
            accelerationWindow.draw(panel);

            Text txt(font);
            txt.setCharacterSize(14); txt.setFillColor(Color::White);
            txt.setString("Aa  = "+to_string(Aa_mag)+
                          "\nAba = "+to_string(Aba_mag)+
                          "\nAb  = "+to_string(Ab_mag));
            txt.setPosition({20,20});
            accelerationWindow.draw(txt);

            Text scaleTxt(font);
            scaleTxt.setCharacterSize(14); scaleTxt.setFillColor(Color::White);
            scaleTxt.setString("Scale: 1 px = "+to_string(1/scale)+" m/s^2");
            scaleTxt.setPosition({20,450});
            accelerationWindow.draw(scaleTxt);

            Text title(font);
            title.setString("Acceleration Diagram");
            title.setCharacterSize(18); title.setPosition({140,20});
            accelerationWindow.draw(title);

            accelerationWindow.display();
        }
    }

    ImGui::SFML::Shutdown();
}