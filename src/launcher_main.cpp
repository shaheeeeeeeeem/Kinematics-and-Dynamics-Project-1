#include <SFML/Graphics.hpp>
#include <windows.h>

int main()
{
    sf::RenderWindow window(sf::VideoMode({400,250}), "Mechanism Simulator");

    sf::Font font("C:/Windows/Fonts/arial.ttf");

    sf::Text title(font);
    title.setString("Mechanism Simulator");
    title.setCharacterSize(24);
    title.setPosition({70,20});

    sf::RectangleShape fourbarButton({200,50});
    fourbarButton.setPosition({100,80});
    fourbarButton.setFillColor(sf::Color(70,70,70));
    fourbarButton.setOutlineThickness(2);
    fourbarButton.setOutlineColor(sf::Color::White);

    sf::RectangleShape sliderButton({200,50});
    sliderButton.setPosition({100,150});
    sliderButton.setFillColor(sf::Color(70,70,70));
    sliderButton.setOutlineThickness(2);
    sliderButton.setOutlineColor(sf::Color::White);

    sf::Text fourbarText(font);
    fourbarText.setString("Four Bar Mechanism");
    fourbarText.setCharacterSize(16);
    fourbarText.setPosition({120,95});

    sf::Text sliderText(font);
    sliderText.setString("Slider Crank Mechanism");
    sliderText.setCharacterSize(16);
    sliderText.setPosition({105,165});

    while(window.isOpen())
    {
        while(auto event = window.pollEvent())
        {
            if(event->is<sf::Event::Closed>())
                window.close();

            if(auto mouse = event->getIf<sf::Event::MouseButtonPressed>())
            {
                sf::Vector2f m(mouse->position.x, mouse->position.y);

                if(fourbarButton.getGlobalBounds().contains(m))
                {
                    ShellExecuteA(NULL, "open", "fourbar.exe", NULL, NULL, SW_SHOWNORMAL);
                    window.close();
                }

                if(sliderButton.getGlobalBounds().contains(m))
                {
                    ShellExecuteA(NULL, "open", "slidercrank.exe", NULL, NULL, SW_SHOWNORMAL);
                    window.close();
                }
            }
        }

        // Mouse position for hover effect
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        sf::Vector2f mouse(mousePos.x, mousePos.y);

        // Hover color change
        if(fourbarButton.getGlobalBounds().contains(mouse))
            fourbarButton.setFillColor(sf::Color(120,120,120));
        else
            fourbarButton.setFillColor(sf::Color(70,70,70));

        if(sliderButton.getGlobalBounds().contains(mouse))
            sliderButton.setFillColor(sf::Color(120,120,120));
        else
            sliderButton.setFillColor(sf::Color(70,70,70));

        window.clear(sf::Color::Black);

        window.draw(title);
        window.draw(fourbarButton);
        window.draw(sliderButton);
        window.draw(fourbarText);
        window.draw(sliderText);

        window.display();
    }
}