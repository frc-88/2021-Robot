package frc.robot.subsystems.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import frc.robot.subsystems.Drive;

public class TunnelServer extends Thread {

    private int port = 0;
    private Drive m_drive;
    private ArrayList<TunnelThread> tunnels;

    public TunnelServer(Drive drive, int port)
    {
        tunnels = new ArrayList<TunnelThread>();
        this.port = port;
        this.m_drive = drive;
    }

    public void update() {
        sendOdometry();
    }

    private void sendOdometry()
    {
        for (int index = tunnels.size() - 1; index >= 0; index--)
        {
            TunnelThread tunnel = tunnels.get(index);
            if (tunnel.isAlive() && tunnel.isOpen()) {
                tunnel.sendOdometry();
            }
        }
    }

    public boolean setCommandIfActive()
    {
        for (int index = tunnels.size() - 1; index >= 0; index--)
        {
            TunnelThread tunnel = tunnels.get(index);
            if (tunnel.isCommandActive()) {
                m_drive.arcadeDrive(tunnel.getSpeedCommand(), tunnel.getTurnCommand());
                return true;
            }
        }
        return false;
    }

    private void cleanUpThreads()
    {
        for (int index = 0; index < tunnels.size(); index++)
        {
            if (!tunnels.get(index).isAlive() || !tunnels.get(index).isOpen()) {
                tunnels.remove(index);
                index--;
            }
        }
    }

    @Override
    public void run()
    {
        ServerSocket serverSocket = null;
        Socket socket = null;

        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        System.out.println("Socket is open");
        try
        {
            while (true) {
                try {
                    socket = serverSocket.accept();
                } catch (IOException e) {
                    System.out.println("I/O error: " + e);
                }
                cleanUpThreads();

                // new threads for a client
                TunnelThread tunnel = new TunnelThread(m_drive, socket);
                tunnels.add(tunnel);
                tunnel.start();
            }
        }
        finally {
            try {
                System.out.println("Closing socket");
                serverSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
