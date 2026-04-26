#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace sf;

constexpr float PI = 3.14159265359f;
constexpr float dt = 0.001f;

struct MechState
{
    float theta2=0, theta3=0, theta4=0;
    float omega2=0, omega3=0, omega4=0;
    float alpha2=0, alpha3=0, alpha4=0;
};

bool isFiniteVec(Vector2f v){ return isfinite(v.x) && isfinite(v.y); }

// ── Grashof helpers ───────────────────────────────────────────────────────────
bool isGrashof(float* lengths)
{
    float t[4]={lengths[0],lengths[1],lengths[2],lengths[3]};
    sort(t,t+4);
    return (t[0]+t[3]) <= (t[1]+t[2]);
}

bool isSpecialGrashof(float* lengths)
{
    float t[4]={lengths[0],lengths[1],lengths[2],lengths[3]};
    sort(t,t+4);
    return fabs((t[0]+t[3])-(t[1]+t[2])) < 1e-4f;
}

bool isStrictGrashof(float* lengths)
{
    float t[4]={lengths[0],lengths[1],lengths[2],lengths[3]};
    sort(t,t+4);
    return (t[0]+t[3]) < (t[1]+t[2]) - 1e-4f;
}

float angleDiff(float a, float b)
{
    float d=a-b;
    while(d> PI) d-=2*PI;
    while(d<=-PI) d+=2*PI;
    return d;
}

bool solvePosition(float a, float b, float c, float d,
                   MechState& s, bool alt,
                   float prevTheta3, float prevTheta4,
                   bool firstSolve=false)
{
    Vector2f A(a*cos(s.theta2), a*sin(s.theta2));
    Vector2f r=Vector2f(d,0)-A;
    float R=sqrt(r.x*r.x+r.y*r.y);
    if(R>(b+c)||R<fabs(b-c)) return false;

    float cosArg=(b*b+R*R-c*c)/(2*b*R);
    cosArg=max(-1.f,min(1.f,cosArg));
    float alpha=acos(cosArg);
    float base=atan2(r.y,r.x);

    float th3A=base+alpha, th3B=base-alpha;
    Vector2f BA(A.x+b*cos(th3A),A.y+b*sin(th3A));
    Vector2f BB(A.x+b*cos(th3B),A.y+b*sin(th3B));
    float th4A=atan2(BA.y,BA.x-d);
    float th4B=atan2(BB.y,BB.x-d);

    if(firstSolve){
        s.theta3=alt?th3B:th3A;
        s.theta4=alt?th4B:th4A;
        return true;
    }

    float errA=fabs(angleDiff(th4A,prevTheta4))*2.f+fabs(angleDiff(th3A,prevTheta3));
    float errB=fabs(angleDiff(th4B,prevTheta4))*2.f+fabs(angleDiff(th3B,prevTheta3));
    if(errA<=errB){s.theta3=th3A;s.theta4=th4A;}
    else          {s.theta3=th3B;s.theta4=th4B;}
    return true;
}

void solveVelocity(float a, float b, float c, MechState& s)
{
    float denom=sin(s.theta3-s.theta4);
    if(fabs(denom)<1e-4f) denom=(denom>=0.f?1e-4f:-1e-4f);
    s.omega3=(a*s.omega2/b)*(sin(s.theta4-s.theta2)/denom);
    s.omega4=(a*s.omega2/c)*(sin(s.theta2-s.theta3)/denom);
}

void solveAcceleration(float a, float b, float c, MechState& s)
{
    float A=c*sin(s.theta4),B=b*sin(s.theta3);
    float D=c*cos(s.theta4),E=b*cos(s.theta3);
    float C=a*s.alpha2*sin(s.theta2)+a*s.omega2*s.omega2*cos(s.theta2)
           +b*s.omega3*s.omega3*cos(s.theta3)-c*s.omega4*s.omega4*cos(s.theta4);
    float F=a*s.alpha2*cos(s.theta2)-a*s.omega2*s.omega2*sin(s.theta2)
           -b*s.omega3*s.omega3*sin(s.theta3)+c*s.omega4*s.omega4*sin(s.theta4);
    float denom=A*E-B*D;
    if(fabs(denom)<1e-6f) return;
    s.alpha3=(C*D-A*F)/denom;
    s.alpha4=(C*E-B*F)/denom;
}

// ── Termination criterion ─────────────────────────────────────────────────────
// Strict Grashof  → 1 full revolution  (2π)
// Special Grashof → 2 full revolutions (4π) OR 2 reversals (whichever first)
// Non-Grashof     → reversal counting only (-1)
float requiredRevolutions(float* lengths)
{
    if(isStrictGrashof(lengths)) return 2.f*PI;
    if(isSpecialGrashof(lengths)) return 4.f*PI;
    return -1.f;
}

int main()
{
    RenderWindow window(VideoMode({1200,800}),"Four Bar Mechanism");
    ImGui::SFML::Init(window);
    Clock deltaClock;
    Font font("C:/Windows/Fonts/arial.ttf");

    float lengths[4]={100,200,150,180};
    int   roles[4]  ={0,1,2,3};

    float couplerDist=40.f, couplerAngleDeg=0.f;
    vector<Vector2f> couplerPath;

    const char* roleNames[]={"Ground (d)","Input (a)","Coupler (b)","Output (c)"};
    float theta2_deg=45, omega2_input=2, alpha2_input=0;
    bool alternateBranch=false, simulationRunning=false, hasBeenSimulated=false;
    bool showGraph=false, collecting=false;
    float theta2_start=0;

    int graphMode=0;
    vector<float> t2Data,t3Data,t4Data,timeData;
    vector<float> w3Data,w4Data,a3Data,a4Data;
    vector<float> vCouplerData,aCouplerData;

    RenderWindow graphWindow,couplerWindow,velocityWindow,accelerationWindow;
    bool showVelocityDiagram=false, showAccelerationDiagram=false;

    MechState s;
    float simTime=0.f;
    bool isFirstSolve=true;
    int  reversalCount=0;
    bool hadFirstReversal=false;

    auto solvePosFirst=[&](float a,float b,float c,float d,MechState& st,bool alt)->bool{
        return solvePosition(a,b,c,d,st,alt,0.f,0.f,true);
    };

    while(window.isOpen())
    {
        while(auto e=window.pollEvent())
        {
            ImGui::SFML::ProcessEvent(window,*e);
            if(e->is<Event::Closed>()) window.close();
        }
        ImGui::SFML::Update(window,deltaClock.restart());

        float a=0,b=0,c=0,d=0;
        for(int i=0;i<4;i++){
            if(roles[i]==0) d=lengths[i];
            if(roles[i]==1) a=lengths[i];
            if(roles[i]==2) b=lengths[i];
            if(roles[i]==3) c=lengths[i];
        }

        bool grashof        = isGrashof(lengths);
        bool specialGrashof = isSpecialGrashof(lengths);
        float reqRev        = requiredRevolutions(lengths);

        // ── Simulation ────────────────────────────────────────────────────────
        if(simulationRunning)
        {
            s.alpha2=alpha2_input;
            float prevTheta3=s.theta3, prevTheta4=s.theta4;

            MechState next=s;
            next.omega2+=next.alpha2*dt;
            next.theta2+=next.omega2*dt;

            if(solvePosition(a,b,c,d,next,alternateBranch,prevTheta3,prevTheta4,false))
            {
                s=next;
                solveVelocity(a,b,c,s);
                solveAcceleration(a,b,c,s);
            }
            else
            {
                // Toggle — reverse
                s.omega2*=-1.f;

                // FIX: count reversals for ALL types, not just !grashof
                // Special Grashof has isGrashof()==true so the old
                // `!grashof` guard silently skipped it.
                if(collecting){
                    reversalCount++;
                    hadFirstReversal=true;
                }

                // Nudge out of toggle zone
                for(int k=0;k<200;k++){
                    MechState nudge=s;
                    nudge.theta2+=nudge.omega2*dt;
                    if(solvePosition(a,b,c,d,nudge,alternateBranch,prevTheta3,prevTheta4,false)){
                        s=nudge; break;
                    }
                }
                solveVelocity(a,b,c,s);
                solveAcceleration(a,b,c,s);
            }

            // ── Collect ───────────────────────────────────────────────────────
            float phi=couplerAngleDeg*PI/180.f;
            Vector2f A_sim(a*cos(s.theta2),-a*sin(s.theta2));
            Vector2f rAP(couplerDist*cos(s.theta3+phi),-couplerDist*sin(s.theta3+phi));
            Vector2f couplerPoint(A_sim.x+rAP.x,A_sim.y+rAP.y);

            Vector2f VA(-a*s.omega2*sin(s.theta2),a*s.omega2*cos(s.theta2));
            Vector2f VP=VA+Vector2f(-s.omega3*rAP.y,s.omega3*rAP.x);
            float vCoupler=sqrt(VP.x*VP.x+VP.y*VP.y);

            Vector2f AA2(-a*s.alpha2*sin(s.theta2)-a*s.omega2*s.omega2*cos(s.theta2),
                         -a*s.alpha2*cos(s.theta2)+a*s.omega2*s.omega2*sin(s.theta2));
            Vector2f AP=AA2+Vector2f(-s.alpha3*rAP.y,s.alpha3*rAP.x)
                            +Vector2f(-s.omega3*s.omega3*rAP.x,-s.omega3*s.omega3*rAP.y);
            float aCoupler=sqrt(AP.x*AP.x+AP.y*AP.y);

            if(collecting)
            {
                simTime+=dt;
                if(isfinite(s.theta3)&&isfinite(s.theta4)&&
                   isfinite(s.omega3)&&isfinite(s.omega4)&&
                   isfinite(s.alpha3)&&isfinite(s.alpha4)&&
                   isFiniteVec(couplerPoint))
                {
                    t2Data.push_back((s.theta2-theta2_start)*180.f/PI);
                    t3Data.push_back(s.theta3*180.f/PI);
                    t4Data.push_back(s.theta4*180.f/PI);
                    couplerPath.push_back(couplerPoint);
                    timeData.push_back(simTime);
                    w3Data.push_back(s.omega3);
                    w4Data.push_back(s.omega4);
                    a3Data.push_back(s.alpha3);
                    a4Data.push_back(s.alpha4);
                    vCouplerData.push_back(vCoupler);
                    aCouplerData.push_back(aCoupler);
                }

                // ── Termination ───────────────────────────────────────────────
                if(reqRev>0.f)
                {
                    float deltaRad=fabs(s.theta2-theta2_start);
                    if(deltaRad>=reqRev) collecting=false;
                    // Special Grashof that oscillates: also stop on 2 reversals
                    if(specialGrashof && reversalCount>=2) collecting=false;
                }
                else
                {
                    // Non-Grashof: 2 reversals = one full oscillation
                    if(reversalCount>=2){
                        float distToStart=fabs(angleDiff(s.theta2,theta2_start));
                        if(distToStart < fabs(s.omega2)*dt*5.f)
                            collecting=false;
                    }
                    if(simTime>60.f) collecting=false;
                }
            }
        }
        else
        {
            if(!hasBeenSimulated)
            {
                s.theta2=theta2_deg*PI/180.f;
                s.omega2=omega2_input; s.alpha2=alpha2_input;
                solvePosFirst(a,b,c,d,s,alternateBranch);
                solveVelocity(a,b,c,s);
                solveAcceleration(a,b,c,s);
                isFirstSolve=false;
            }
        }

        window.clear(Color::Black);

        // ── Draw ──────────────────────────────────────────────────────────────
        float maxLen=max({a,b,c,d});
        float drawScale=1.f;
        if(maxLen<50.f)       drawScale=8.f;
        else if(maxLen<100.f) drawScale=4.f;
        else if(maxLen<200.f) drawScale=2.f;

        float scaledD=d*drawScale;
        Vector2f O2(400,600);
        Vector2f O4=O2+Vector2f(scaledD,0);
        Vector2f A(O2.x+a*drawScale*cos(s.theta2),O2.y-a*drawScale*sin(s.theta2));
        Vector2f B(A.x +b*drawScale*cos(s.theta3),A.y -b*drawScale*sin(s.theta3));

        auto safeDrawLine=[&](Vector2f p1,Vector2f p2,Color col=Color::White){
            if(!isFiniteVec(p1)||!isFiniteVec(p2)) return;
            Vertex l[2]; l[0]={p1,col}; l[1]={p2,col};
            window.draw(l,2,PrimitiveType::Lines);
        };
        auto drawJoint=[&](Vector2f p,Color col=Color::White){
            if(!isFiniteVec(p)) return;
            CircleShape j(6); j.setOrigin({6,6});
            j.setFillColor(col); j.setPosition(p); window.draw(j);
        };
        auto drawText=[&](string str,Vector2f pos){
            Text t(font); t.setString(str); t.setCharacterSize(14);
            t.setFillColor(Color::Yellow); t.setPosition(pos); window.draw(t);
        };
        auto mid=[](Vector2f p1,Vector2f p2){
            return Vector2f((p1.x+p2.x)/2,(p1.y+p2.y)/2);
        };

        safeDrawLine(O2,O4); safeDrawLine(O2,A);
        safeDrawLine(A,B);   safeDrawLine(B,O4);
        drawJoint(O2); drawJoint(O4); drawJoint(A); drawJoint(B);
        drawText("O2",O2+Vector2f(-20,-20)); drawText("O4",O4+Vector2f(10,-20));
        drawText("A", A +Vector2f( 10,-10)); drawText("B", B +Vector2f(10,-10));
        drawText("d",mid(O2,O4)); drawText("a",mid(O2,A));
        drawText("b",mid(A,B));   drawText("c",mid(B,O4));

        if(!isStrictGrashof(lengths)&&simulationRunning){
            Text dir(font);
            dir.setString(s.omega2>0?">> Forward":"<< Reverse");
            dir.setCharacterSize(14); dir.setFillColor(Color::Cyan);
            dir.setPosition({410.f,20.f}); window.draw(dir);
        }

        // ── Role validation ───────────────────────────────────────────────────
        int roleCount[4]={0,0,0,0};
        for(int i=0;i<4;i++) roleCount[roles[i]]++;
        bool rolesValid=true;
        for(int i=0;i<4;i++) if(roleCount[i]!=1) rolesValid=false;

        // ── Grashof classification ────────────────────────────────────────────
        float temp[4]={lengths[0],lengths[1],lengths[2],lengths[3]};
        sort(temp,temp+4);
        float S=temp[0],L=temp[3],P=temp[1],Q=temp[2];
        float left=S+L, right=P+Q;

        string grashofClass;
        if(fabs(left-right)<1e-4f)  grashofClass="Class 2 (Special Grashof)";
        else if(left<right)         grashofClass="Class 1 (Grashof)";
        else                        grashofClass="Class 3 (Non-Grashof)";

        int groundIndex=-1,shortestIndex=0,longestIndex=0;
        for(int i=0;i<4;i++) if(roles[i]==0) groundIndex=i;
        for(int i=1;i<4;i++){
            if(lengths[i]<lengths[shortestIndex]) shortestIndex=i;
            if(lengths[i]>lengths[longestIndex])  longestIndex=i;
        }
        string mechType;
        if(left>right) mechType="Triple Rocker (Non-Grashof)";
        else if(fabs(left-right)<1e-4f) mechType="Change-Point (Special Grashof)";
        else{
            if(groundIndex==shortestIndex) mechType="Double Crank";
            else if(groundIndex==(shortestIndex+1)%4||
                    groundIndex==(shortestIndex+3)%4) mechType="Crank Rocker";
            else mechType="Grashof Double Rocker";
        }

        // ── ImGui ─────────────────────────────────────────────────────────────
        ImGui::Begin("Four Bar Configuration");
        if(simulationRunning) ImGui::BeginDisabled();
        for(int i=0;i<4;i++){
            ImGui::PushID(i);
            if(ImGui::InputFloat("Length",&lengths[i])){
                if(lengths[i]<=0.f) lengths[i]=10.f;
                hasBeenSimulated=false; isFirstSolve=true;
            }
            ImGui::SameLine();
            if(ImGui::Combo("Role",&roles[i],roleNames,4)){
                hasBeenSimulated=false; isFirstSolve=true;
            }
            ImGui::PopID();
        }
        if(ImGui::SliderFloat("Theta2 (deg)",&theta2_deg,0,360)){hasBeenSimulated=false;isFirstSolve=true;}
        if(ImGui::InputFloat("Omega2",&omega2_input)){hasBeenSimulated=false;isFirstSolve=true;}
        if(ImGui::InputFloat("Alpha2",&alpha2_input)) hasBeenSimulated=false;
        ImGui::InputFloat("Coupler Distance",&couplerDist);
        ImGui::SliderFloat("Coupler Angle",&couplerAngleDeg,0,360);
        if(simulationRunning) ImGui::EndDisabled();

        if(!rolesValid)
            ImGui::TextColored(ImVec4(1,0,0,1),"Invalid: each role must be used exactly once.");

        if(rolesValid){
            if(ImGui::Button(simulationRunning?"Stop":"Start")){
                simulationRunning=!simulationRunning;
                if(simulationRunning){
                    hasBeenSimulated=true;
                    s.omega2=fabs(s.omega2);
                    t2Data.clear();t3Data.clear();t4Data.clear();
                    timeData.clear();w3Data.clear();w4Data.clear();
                    a3Data.clear();a4Data.clear();
                    couplerPath.clear();vCouplerData.clear();aCouplerData.clear();
                    simTime=0.f; theta2_start=s.theta2;
                    reversalCount=0; hadFirstReversal=false;
                    collecting=true;
                }else collecting=false;
            }
        }else{ ImGui::BeginDisabled(); ImGui::Button("Start"); ImGui::EndDisabled(); }

        if(ImGui::Button("Show Graph")){
            showGraph=true;
            string title;
            if(graphMode==0) title="Theta vs Delta Theta2";
            else if(graphMode==1) title="Theta vs Time";
            else if(graphMode==2) title="Omega vs Time";
            else if(graphMode==3) title="Alpha vs Time";
            else if(graphMode==4) title="Coupler Velocity vs Theta2";
            else title="Coupler Acceleration vs Theta2";
            graphWindow.create(VideoMode({900,650}),title);
        }
        const char* graphModes[]={"Theta vs Theta2","Theta vs Time","Omega vs Time",
            "Alpha vs Time","Coupler Velocity vs Theta2","Coupler Acceleration vs Theta2"};
        ImGui::Combo("Graph Type",&graphMode,graphModes,6);

        static bool showCouplerGraph=false;
        if(ImGui::Button("Coupler Curve")) showCouplerGraph=true;
        if(ImGui::Button("Velocity Diagram")){
            showVelocityDiagram=true;
            if(!velocityWindow.isOpen())
                velocityWindow.create(VideoMode({500,500}),"Velocity Diagram");
        }
        if(ImGui::Button("Acceleration Diagram")){
            showAccelerationDiagram=true;
            if(!accelerationWindow.isOpen())
                accelerationWindow.create(VideoMode({500,500}),"Acceleration Diagram");
        }
        if(ImGui::Button("Reset")){
            simulationRunning=false; collecting=false; hasBeenSimulated=false;
            isFirstSolve=true; reversalCount=0; hadFirstReversal=false;
            t2Data.clear();t3Data.clear();t4Data.clear();
            timeData.clear();w3Data.clear();w4Data.clear();
            a3Data.clear();a4Data.clear();
            couplerPath.clear();vCouplerData.clear();aCouplerData.clear();
            simTime=0.f;
            s.theta2=theta2_deg*PI/180.f;
            s.omega2=omega2_input; s.alpha2=alpha2_input;
            solvePosFirst(a,b,c,d,s,alternateBranch);
            solveVelocity(a,b,c,s); solveAcceleration(a,b,c,s);
            isFirstSolve=false;
        }

        ImGui::Separator();
        ImGui::Text("%s",grashofClass.c_str());
        ImGui::Text("%s",mechType.c_str());
        if(collecting){
            if(reqRev>0.f){
                float deltaRad=fabs(s.theta2-theta2_start);
                float pct=min(100.f,deltaRad/reqRev*100.f);
                ImGui::Text("Collecting: %.0f%%",pct);
            }else ImGui::Text("Collecting (reversals: %d/2)",reversalCount);
        }
        ImGui::Separator();
        ImGui::Text("Position Analysis");
        ImGui::Text("Theta2: %.2f",s.theta2*180/PI);
        ImGui::Text("Theta3: %.2f",s.theta3*180/PI);
        ImGui::Text("Theta4: %.2f",s.theta4*180/PI);
        ImGui::Separator();
        ImGui::Text("Angular Velocity Analysis");
        ImGui::Text("Omega2: %.3f",s.omega2);
        ImGui::Text("Omega3: %.3f",s.omega3);
        ImGui::Text("Omega4: %.3f",s.omega4);
        ImGui::Separator();
        ImGui::Text("Angular Acceleration Analysis");
        ImGui::Text("Alpha2: %.3f",s.alpha2);
        ImGui::Text("Alpha3: %.3f",s.alpha3);
        ImGui::Text("Alpha4: %.3f",s.alpha4);
        if(ImGui::Checkbox("Alternate Branch",&alternateBranch)){
            hasBeenSimulated=false; isFirstSolve=true;
        }
        ImGui::End();
        ImGui::SFML::Render(window);
        window.display();

        // ── Shared helpers ────────────────────────────────────────────────────
        auto drawLabel=[&](RenderWindow& win,string text,Vector2f pos){
            Text t(font); t.setString(text); t.setCharacterSize(14);
            t.setFillColor(Color::White); t.setPosition(pos); win.draw(t);
        };
        auto drawVec=[&](RenderWindow& win,Vector2f start,Vector2f vec,float scale,Color col){
            if(!isFiniteVec(start)||!isFiniteVec(vec)) return;
            Vector2f end=start+vec*scale;
            Vertex line[2]; line[0]={start,col}; line[1]={end,col};
            win.draw(line,2,PrimitiveType::Lines);
            float sz=10.f,ang=atan2(end.y-start.y,end.x-start.x);
            Vector2f a1(end.x-sz*cos(ang-PI/6),end.y-sz*sin(ang-PI/6));
            Vector2f a2(end.x-sz*cos(ang+PI/6),end.y-sz*sin(ang+PI/6));
            Vertex arr[4]; arr[0]={end,col};arr[1]={a1,col};arr[2]={end,col};arr[3]={a2,col};
            win.draw(arr,4,PrimitiveType::Lines);
        };

        // ── Graph window ──────────────────────────────────────────────────────
        if(showGraph&&graphWindow.isOpen())
        {
            while(auto e=graphWindow.pollEvent())
                if(e->is<Event::Closed>()) graphWindow.close();
            graphWindow.clear(Color::Black);

            if(!t2Data.empty())
            {
                float margin=100,width=650,height=450;
                Vector2f origin(margin,550);
                const vector<float>*xData=nullptr,*y1=nullptr,*y2=nullptr;
                string yLabelText;
                if(graphMode==0){xData=&t2Data;y1=&t3Data;y2=&t4Data;yLabelText="Theta3/Theta4 (deg)";}
                else if(graphMode==1){xData=&timeData;y1=&t3Data;y2=&t4Data;yLabelText="Theta3/Theta4 (deg)";}
                else if(graphMode==2){xData=&timeData;y1=&w3Data;y2=&w4Data;yLabelText="Omega3/Omega4 (rad/s)";}
                else if(graphMode==3){xData=&timeData;y1=&a3Data;y2=&a4Data;yLabelText="Alpha3/Alpha4 (rad/s^2)";}
                else if(graphMode==4){xData=&t2Data;y1=&vCouplerData;y2=nullptr;yLabelText="Coupler Velocity (mm/s)";}
                else{xData=&t2Data;y1=&aCouplerData;y2=nullptr;yLabelText="Coupler Acceleration (mm/s^2)";}

                size_t dataSize=xData->size();
                if(dataSize==0){graphWindow.display();continue;}

                float minY,maxY,scaleX;
                if(graphMode==0||graphMode==1){minY=-360.f;maxY=360.f;}
                else{
                    minY=maxY=(*y1)[0];
                    for(size_t i=0;i<dataSize;i++){
                        minY=min(minY,(*y1)[i]);maxY=max(maxY,(*y1)[i]);
                        if(y2){minY=min(minY,(*y2)[i]);maxY=max(maxY,(*y2)[i]);}
                    }
                    float pad=(maxY-minY)*0.1f;minY-=pad;maxY+=pad;
                }
                if(fabs(maxY-minY)<1e-5f){minY-=1.f;maxY+=1.f;}

                float rangeY=maxY-minY,scaleY=height/rangeY;
                float maxX;
                if(graphMode==0||graphMode==4||graphMode==5)
                    maxX=specialGrashof?720.f:360.f;
                else
                    maxX=timeData.empty()?1.f:timeData.back();
                if(maxX<=0) maxX=1.f;
                scaleX=width/maxX;

                Vertex xa[2]={{origin,Color::White},{{origin.x+width,origin.y},Color::White}};
                Vertex ya[2]={{origin,Color::White},{{origin.x,origin.y-height},Color::White}};
                graphWindow.draw(xa,2,PrimitiveType::Lines);
                graphWindow.draw(ya,2,PrimitiveType::Lines);

                float yStep=(graphMode==0||graphMode==1)?30.f:
                            (graphMode==2)?1.f:(graphMode==3)?2.f:(maxY-minY)/10.f;
                if(yStep<1e-5f) yStep=1.f;
                for(float yv=floor(minY/yStep)*yStep;yv<=maxY;yv+=yStep){
                    float y=origin.y-(yv-minY)*scaleY;
                    Vertex g[2]={{Vector2f(origin.x,y),Color(70,70,70)},{Vector2f(origin.x+width,y),Color(70,70,70)}};
                    graphWindow.draw(g,2,PrimitiveType::Lines);
                    Text lbl(font);ostringstream ss;ss<<(int)yv;
                    lbl.setString(ss.str());lbl.setCharacterSize(12);lbl.setFillColor(Color::White);
                    lbl.setPosition({origin.x-55.f,y-8.f});graphWindow.draw(lbl);
                }
                float xMax=maxX;
                float xStep=(graphMode==0||graphMode==4||graphMode==5)?(specialGrashof?60.f:30.f):0.5f;
                for(float xv=0;xv<=xMax;xv+=xStep){
                    float x=origin.x+xv*scaleX;
                    Vertex g[2]={{Vector2f(x,origin.y),Color(70,70,70)},{Vector2f(x,origin.y-height),Color(70,70,70)}};
                    graphWindow.draw(g,2,PrimitiveType::Lines);
                    Text lbl(font);ostringstream ss;
                    if(graphMode==0||graphMode==4||graphMode==5) ss<<(int)xv;
                    else ss<<fixed<<setprecision(1)<<xv;
                    lbl.setString(ss.str());lbl.setCharacterSize(12);lbl.setFillColor(Color::White);
                    lbl.setPosition({x-15.f,origin.y+10.f});graphWindow.draw(lbl);
                }
                Text xl(font),yl2(font);
                xl.setString((graphMode==0||graphMode==4||graphMode==5)?"Delta Theta2 (deg)":"Time (s)");
                xl.setCharacterSize(14);xl.setFillColor(Color::White);
                xl.setPosition({origin.x+width/2.f-60.f,origin.y+40.f});graphWindow.draw(xl);
                yl2.setString(yLabelText);yl2.setCharacterSize(14);yl2.setFillColor(Color::White);
                yl2.setPosition({origin.x-90.f,origin.y-height-30.f});graphWindow.draw(yl2);

                VertexArray c3(PrimitiveType::LineStrip,dataSize);
                VertexArray c4(PrimitiveType::LineStrip);
                if(y2) c4.resize(dataSize);
                for(size_t i=0;i<dataSize;i++){
                    float x=origin.x+(*xData)[i]*scaleX;
                    float y3=origin.y-((*y1)[i]-minY)*scaleY;
                    c3[i]={Vector2f(x,y3),Color::Green};
                    if(y2){float y4=origin.y-((*y2)[i]-minY)*scaleY;c4[i]={Vector2f(x,y4),Color::Red};}
                }
                graphWindow.draw(c3);
                if(y2) graphWindow.draw(c4);

                Vector2i mp=Mouse::getPosition(graphWindow);
                Vector2f mouse=graphWindow.mapPixelToCoords(mp);
                if(mouse.x>=origin.x&&mouse.x<=origin.x+width&&
                   mouse.y<=origin.y&&mouse.y>=origin.y-height&&!xData->empty())
                {
                    float xValue=(mouse.x-origin.x)/scaleX;
                    if(xValue>=0.f&&xValue<=xData->back()){
                        size_t idx=0;
                        for(size_t i=1;i<dataSize;i++)
                            if(fabs((*xData)[i]-xValue)<fabs((*xData)[idx]-xValue)) idx=i;
                        xValue=(*xData)[idx];
                        float y3h=(*y1)[idx],y4h=y2?(*y2)[idx]:0.f;
                        Vertex cur[2]={{Vector2f(mouse.x,origin.y),Color(100,100,100)},
                                       {Vector2f(mouse.x,origin.y-height),Color(100,100,100)}};
                        graphWindow.draw(cur,2,PrimitiveType::Lines);
                        Text info(font);info.setCharacterSize(13);info.setFillColor(Color::White);
                        string txt;
                        if(graphMode==0) txt="DTheta2: "+to_string((int)xValue)+" deg\nTheta3: "+to_string((int)y3h)+" deg\nTheta4: "+to_string((int)y4h)+" deg";
                        else if(graphMode==1) txt="Time: "+to_string(xValue)+" s\nTheta3: "+to_string((int)y3h)+"\nTheta4: "+to_string((int)y4h);
                        else if(graphMode==2) txt="Time: "+to_string(xValue)+" s\nOmega3: "+to_string(y3h)+"\nOmega4: "+to_string(y4h);
                        else if(graphMode==3) txt="Time: "+to_string(xValue)+" s\nAlpha3: "+to_string(y3h)+"\nAlpha4: "+to_string(y4h);
                        else if(graphMode==4) txt="Theta2: "+to_string(xValue)+"\nVcoupler: "+to_string(y3h);
                        else txt="Theta2: "+to_string(xValue)+"\nAcoupler: "+to_string(y3h);
                        info.setString(txt);
                        info.setPosition({origin.x+width-170.f,origin.y-height-60.f});
                        graphWindow.draw(info);
                    }
                }
                string l1,l2;
                if(graphMode<=1){l1="Theta3";l2="Theta4";}
                else if(graphMode==2){l1="Omega3";l2="Omega4";}
                else if(graphMode==3){l1="Alpha3";l2="Alpha4";}
                else if(graphMode==4) l1="Coupler Velocity";
                else l1="Coupler Acceleration";
                Text lg1(font);lg1.setString(l1);lg1.setFillColor(Color::Green);lg1.setCharacterSize(14);
                lg1.setPosition({origin.x+width-170.f,origin.y-height+10.f});graphWindow.draw(lg1);
                if(y2){Text lg2(font);lg2.setString(l2);lg2.setFillColor(Color::Red);lg2.setCharacterSize(14);
                       lg2.setPosition({origin.x+width-170.f,origin.y-height+30.f});graphWindow.draw(lg2);}
            }
            graphWindow.display();
        }

        // ── Coupler curve ─────────────────────────────────────────────────────
        if(showCouplerGraph&&!couplerWindow.isOpen())
            couplerWindow.create(VideoMode({700,700}),"Coupler Curve");
        if(couplerWindow.isOpen())
        {
            while(auto ev=couplerWindow.pollEvent())
                if(ev->is<Event::Closed>()){couplerWindow.close();showCouplerGraph=false;}
            couplerWindow.clear(Color::Black);
            if(couplerPath.size()>=2)
            {
                float mnX=1e9,mxX=-1e9,mnY=1e9,mxY=-1e9;
                for(auto& p:couplerPath){mnX=min(mnX,p.x);mxX=max(mxX,p.x);mnY=min(mnY,p.y);mxY=max(mxY,p.y);}
                float w=500.f,h=500.f; Vector2f org(100.f,600.f);
                float sx=(mxX-mnX>1e-5f)?w/(mxX-mnX):1.f;
                float sy=(mxY-mnY>1e-5f)?h/(mxY-mnY):1.f;
                for(float gx=org.x;gx<=org.x+w;gx+=50.f){
                    Vertex l[2]={{Vector2f(gx,org.y),Color(60,60,60)},{Vector2f(gx,org.y-h),Color(60,60,60)}};
                    couplerWindow.draw(l,2,PrimitiveType::Lines);}
                for(float gy=org.y;gy>=org.y-h;gy-=50.f){
                    Vertex l[2]={{Vector2f(org.x,gy),Color(60,60,60)},{Vector2f(org.x+w,gy),Color(60,60,60)}};
                    couplerWindow.draw(l,2,PrimitiveType::Lines);}
                Vertex xa[2]={{org,Color::White},{{org.x+w,org.y},Color::White}};
                Vertex ya[2]={{org,Color::White},{{org.x,org.y-h},Color::White}};
                couplerWindow.draw(xa,2,PrimitiveType::Lines);
                couplerWindow.draw(ya,2,PrimitiveType::Lines);
                for(int i=0;i<=10;i++){
                    float x=org.x+i*(w/10);
                    Vertex tk[2]={{Vector2f(x,org.y),Color::White},{Vector2f(x,org.y+6),Color::White}};
                    couplerWindow.draw(tk,2,PrimitiveType::Lines);
                    Text lbl(font);lbl.setCharacterSize(11);lbl.setFillColor(Color::White);
                    lbl.setString(to_string((int)(mnX+(mxX-mnX)*i/10.f)));
                    lbl.setPosition({x-10,org.y+8});couplerWindow.draw(lbl);}
                for(int i=0;i<=10;i++){
                    float y=org.y-i*(h/10);
                    Vertex tk[2]={{Vector2f(org.x,y),Color::White},{Vector2f(org.x-6,y),Color::White}};
                    couplerWindow.draw(tk,2,PrimitiveType::Lines);
                    Text lbl(font);lbl.setCharacterSize(11);lbl.setFillColor(Color::White);
                    lbl.setString(to_string((int)(mnY+(mxY-mnY)*i/10.f)));
                    lbl.setPosition({org.x-42,y-6});couplerWindow.draw(lbl);}
                Text xl(font);xl.setString("X Position (mm)");xl.setCharacterSize(14);
                xl.setFillColor(Color::White);xl.setPosition({org.x+w/2.f-50.f,org.y+22.f});couplerWindow.draw(xl);
                Text yl2(font);yl2.setString("Y Position (mm)");yl2.setCharacterSize(14);
                yl2.setFillColor(Color::White);yl2.setRotation(degrees(-90.f));
                yl2.setPosition({org.x-75.f,org.y-h/2.f+40.f});couplerWindow.draw(yl2);
                for(size_t i=1;i<couplerPath.size();i++){
                    float x1=org.x+(couplerPath[i-1].x-mnX)*sx,y1=org.y-(couplerPath[i-1].y-mnY)*sy;
                    float x2=org.x+(couplerPath[i].x-mnX)*sx,  y2=org.y-(couplerPath[i].y-mnY)*sy;
                    if(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))>50.f) continue;
                    Vertex ln[2]={{Vector2f(x1,y1),Color::Magenta},{Vector2f(x2,y2),Color::Magenta}};
                    couplerWindow.draw(ln,2,PrimitiveType::Lines);}
            }
            couplerWindow.display();
        }

        // ── Velocity diagram ──────────────────────────────────────────────────
        Vector2f VA(-a*s.omega2*sin(s.theta2),a*s.omega2*cos(s.theta2));
        Vector2f VBA(-b*s.omega3*sin(s.theta3),b*s.omega3*cos(s.theta3));
        Vector2f VB=VA+VBA;
        if(showVelocityDiagram&&velocityWindow.isOpen())
        {
            while(auto e=velocityWindow.pollEvent())
                if(e->is<Event::Closed>()) velocityWindow.close();
            velocityWindow.clear(Color::Black);
            Vector2f org(250,250);
            float Vam=sqrt(VA.x*VA.x+VA.y*VA.y),Vbam=sqrt(VBA.x*VBA.x+VBA.y*VBA.y),Vbm=sqrt(VB.x*VB.x+VB.y*VB.y);
            float mx=max(Vam,max(Vbam,Vbm));
            float sc=(mx>1e-5f&&isfinite(mx))?120.f/mx:1.f;
            RectangleShape pnl;pnl.setSize({175,72});pnl.setPosition({10,10});
            pnl.setFillColor(Color(30,30,30,200));velocityWindow.draw(pnl);
            Text txt(font);txt.setCharacterSize(13);txt.setFillColor(Color::White);
            txt.setString("Va  = "+to_string(Vam)+"\nVba = "+to_string(Vbam)+"\nVb  = "+to_string(Vbm));
            txt.setPosition({15,15});velocityWindow.draw(txt);
            drawVec(velocityWindow,org,VA,sc,Color::Blue);
            drawVec(velocityWindow,org+VA*sc,VBA,sc,Color::Green);
            drawVec(velocityWindow,org,VB,sc,Color::Red);
            drawLabel(velocityWindow,"Va", (org+org+VA*sc)*0.5f+Vector2f(5,-10));
            drawLabel(velocityWindow,"Vba",(org+VA*sc+org+VA*sc+VBA*sc)*0.5f+Vector2f(5,-10));
            drawLabel(velocityWindow,"Vb", (org+org+VB*sc)*0.5f+Vector2f(5,-10));
            Text scT(font);scT.setCharacterSize(13);scT.setFillColor(Color::White);
            scT.setString("Scale: 1px = "+to_string(1.f/sc)+" m/s");
            scT.setPosition({15,450});velocityWindow.draw(scT);
            Text ttl(font);ttl.setString("Velocity Diagram");ttl.setCharacterSize(16);
            ttl.setPosition({140,15});velocityWindow.draw(ttl);
            velocityWindow.display();
        }

        // ── Acceleration diagram ──────────────────────────────────────────────
        Vector2f AA(-a*s.alpha2*sin(s.theta2)-a*s.omega2*s.omega2*cos(s.theta2),
                    -a*s.alpha2*cos(s.theta2)+a*s.omega2*s.omega2*sin(s.theta2));
        Vector2f ABA(-b*s.alpha3*sin(s.theta3)-b*s.omega3*s.omega3*cos(s.theta3),
                     -b*s.alpha3*cos(s.theta3)+b*s.omega3*s.omega3*sin(s.theta3));
        Vector2f AB=AA+ABA;
        if(showAccelerationDiagram&&accelerationWindow.isOpen())
        {
            while(auto e=accelerationWindow.pollEvent())
                if(e->is<Event::Closed>()) accelerationWindow.close();
            accelerationWindow.clear(Color::Black);
            Vector2f org(250,250);
            float Aam=sqrt(AA.x*AA.x+AA.y*AA.y),Abam=sqrt(ABA.x*ABA.x+ABA.y*ABA.y),Abm=sqrt(AB.x*AB.x+AB.y*AB.y);
            float mx=max(Aam,max(Abam,Abm));
            float sc=(mx>1e-5f&&isfinite(mx))?150.f/mx:1.f;
            RectangleShape pnl;pnl.setSize({180,72});pnl.setPosition({10,10});
            pnl.setFillColor(Color(30,30,30,200));accelerationWindow.draw(pnl);
            Text txt(font);txt.setCharacterSize(13);txt.setFillColor(Color::White);
            txt.setString("Aa  = "+to_string(Aam)+"\nAba = "+to_string(Abam)+"\nAb  = "+to_string(Abm));
            txt.setPosition({15,15});accelerationWindow.draw(txt);
            drawVec(accelerationWindow,org,AA,sc,Color::Blue);
            drawVec(accelerationWindow,org+AA*sc,ABA,sc,Color::Green);
            drawVec(accelerationWindow,org,AB,sc,Color::Red);
            drawLabel(accelerationWindow,"Aa", (org+org+AA*sc)*0.5f+Vector2f(5,-10));
            drawLabel(accelerationWindow,"Aba",(org+AA*sc+org+AA*sc+ABA*sc)*0.5f+Vector2f(5,-10));
            drawLabel(accelerationWindow,"Ab", (org+org+AB*sc)*0.5f+Vector2f(5,-10));
            Text scT(font);scT.setCharacterSize(13);scT.setFillColor(Color::White);
            scT.setString("Scale: 1px = "+to_string(1.f/sc)+" m/s^2");
            scT.setPosition({15,450});accelerationWindow.draw(scT);
            Text ttl(font);ttl.setString("Acceleration Diagram");ttl.setCharacterSize(16);
            ttl.setPosition({120,15});accelerationWindow.draw(ttl);
            accelerationWindow.display();
        }
    }

    ImGui::SFML::Shutdown();
    return 0;
}
