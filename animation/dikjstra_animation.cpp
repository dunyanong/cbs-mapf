#include <SFML/Graphics.hpp>
#include "search_dijkstra.hpp"
#include "graph.hpp"

int main()
{
    // Create the main window
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Dijkstra Animation");

    // Create a graph and add vertices and edges
    raplab::SparseGraph graph;
    graph.AddVertex(0);
    graph.AddVertex(1);
    graph.AddVertex(2);
    graph.AddVertex(3);
    graph.AddVertex(4);
    graph.AddEdge(0, 1, std::vector<double>({10}));
    graph.AddEdge(1, 2, std::vector<double>({20}));
    graph.AddEdge(2, 3, std::vector<double>({30}));
    graph.AddEdge(3, 4, std::vector<double>({40}));

    // Perform Dijkstra's algorithm
    raplab::Dijkstra dijkstra;
    dijkstra.SetGraphPtr(&graph);
    dijkstra.PathFinding(0, 4);

    // Get the path and distances
    auto path = dijkstra.GetPath(4);
    auto distances = dijkstra.GetDistAll();

    // Create visual elements for the graph
    std::vector<sf::CircleShape> nodes;
    std::vector<sf::Text> labels;
    sf::Font font;
    if (!font.openFromFile("/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/animation/arial.ttf")) {
        std::cerr << "Failed to load font file!" << std::endl;
        return -1; // Exit if the font file cannot be loaded
    }

    for (size_t i = 0; i < graph.NumVertex(); ++i)
    {
        sf::CircleShape node(20);
        node.setFillColor(sf::Color::Blue);
        node.setPosition(sf::Vector2f(100 + i * 100, 300));
        nodes.push_back(node);

        const sf::Font font("/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/animation/arial.ttf");
        // Create a text which uses our font
        sf::Text text1(font);
        text1.setCharacterSize(30);
        text1.setStyle(sf::Text::Regular);

        labels.push_back(text1);
    }

    // Create lines for edges
    std::vector<sf::VertexArray> edges;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        sf::VertexArray edge(sf::PrimitiveType::Lines, 2);
        edge[0].position = nodes[path[i]].getPosition() + sf::Vector2f(20, 20);
        edge[1].position = nodes[path[i + 1]].getPosition() + sf::Vector2f(20, 20);
        edge[0].color = sf::Color::White;
        edge[1].color = sf::Color::White;
        edges.push_back(edge);
    }

    // Create an agent
    sf::CircleShape agent(10);
    agent.setFillColor(sf::Color::Red);
    agent.setPosition(nodes[path[0]].getPosition() + sf::Vector2f(20, 20));

    size_t currentStep = 0;
    sf::Clock clock;

    // Start the game loop
    while (window.isOpen())
    {
        // Process events
        while (const auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Clear screen
        window.clear();

        // Draw nodes and labels
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            window.draw(nodes[i]);
            window.draw(labels[i]);
        }

        // Draw edges
        for (const auto &edge : edges)
            window.draw(edge);

        // Animate the agent's movement along the path
        if (currentStep < path.size() - 1)
        {
            float elapsed = clock.getElapsedTime().asSeconds();
            if (elapsed > 0.5f) // Move every 0.5 seconds
            {
                currentStep++;
                agent.setPosition(nodes[path[currentStep]].getPosition() + sf::Vector2f(20, 20));
                nodes[path[currentStep]].setFillColor(sf::Color::Green); // Highlight visited node
                edges[currentStep - 1][0].color = sf::Color::Green;     // Highlight edge
                edges[currentStep - 1][1].color = sf::Color::Green;
                clock.restart();
            }
        }

        // Draw the agent
        window.draw(agent);

        // Update the window
        window.display();
    }

    return 0;
}