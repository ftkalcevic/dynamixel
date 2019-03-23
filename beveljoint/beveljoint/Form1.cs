using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using dynamixel_sdk;
using System.Runtime.InteropServices;

namespace beveljoint
{
	public partial class Form1 : Form
	{
        private const int COMM_PORT = 21;
        private const int COMM_BAUD = 252;
        private const int COMM_SUCCESS = 0;

        private const int P_RETURN_DELAY_L = 5;
        private const int P_CW_LIMIT_L = 6;
        private const int P_CCW_LIMIT_L = 8;
        private const int P_PRESENT_POSITION_L = 36;
        private const int P_PRESENT_SPEED_L = 38;
        private const int P_PRESENT_LOAD_L = 40;
        private const int P_PRESENT_VOLTAGE = 42;
        private const int P_PRESENT_TEMPERATURE = 43;
        private const int P_REGISTERED = 44;
        private const int P_MOVING = 46;
        private const int P_GOAL_POSITION_L = 30;
        private const int P_MOVING_SPEED_L = 32;
        private const int P_TORQUE_ENABLE_L = 24;
        private Timer m_timer;
        private Timer m_animateTimer;
        bool bMoving = false;
        int port_num;

        public Form1()
		{
			InitializeComponent();

            grid.Rows.Add(2);

            m_timer = new Timer();
            m_timer.Interval = 100;
            m_timer.Tick += M_timer_Tick;
            m_timer.Start();

            port_num = dynamixel.portHandler("COM21");
            dynamixel.packetHandler();
            if (!dynamixel.openPort(port_num))
            {
                MessageBox.Show("Failed to connect to USB2Dynamixel");
            }
            dynamixel.setBaudRate(port_num, 3000000);

            dynamixel.write1ByteTxRx(port_num, 1, 2, P_TORQUE_ENABLE_L, 0);
            dynamixel.write1ByteTxRx(port_num, 1, 3, P_TORQUE_ENABLE_L, 0);

            dynamixel.write2ByteTxRx(port_num, 1, 2, P_MOVING_SPEED_L, 0);
            dynamixel.write2ByteTxRx(port_num, 1, 3, P_MOVING_SPEED_L, 0);

            dynamixel.write2ByteTxRx(port_num, 1, 2, P_CW_LIMIT_L, 0);
            dynamixel.write2ByteTxRx(port_num, 1, 3, P_CW_LIMIT_L, 0);

            dynamixel.write2ByteTxRx(port_num, 1, 2, P_CCW_LIMIT_L, 4095);
            dynamixel.write2ByteTxRx(port_num, 1, 3, P_CCW_LIMIT_L, 4095);

            dynamixel.write1ByteTxRx(port_num, 1, 2, P_TORQUE_ENABLE_L, 1);
            dynamixel.write1ByteTxRx(port_num, 1, 3, P_TORQUE_ENABLE_L, 1);

        }



        private void M_timer_Tick(object sender, EventArgs e)
        {

            for (byte i = 2; i <= 3; i++)
            {
                int PresentPosition = dynamixel.read2ByteTxRx(port_num, 1, i, P_PRESENT_POSITION_L);
                int n = dynamixel.read2ByteTxRx(port_num, 1, i, P_PRESENT_SPEED_L);
                double PresentSpeed = n; //(double)(n >= 1024 ? -(n - 1024) : n ) * 0.11;
                n = dynamixel.read2ByteTxRx(port_num, 1, i, P_PRESENT_LOAD_L);
                double PresentLoad = n;  //(double)(n >= 1024 ? -(n - 1024) : n) / 1024.0 * 100.0;
                double PresentVoltage = (double)(dynamixel.read2ByteTxRx(port_num, 1, i, P_PRESENT_VOLTAGE)) / 10.0;
                int PresentTempterature = dynamixel.read2ByteTxRx(port_num, 1, i, P_PRESENT_TEMPERATURE);
                int Registered = dynamixel.read2ByteTxRx(port_num, 1, i, P_REGISTERED);
                int Moving = dynamixel.read2ByteTxRx(port_num, 1, i, P_MOVING);
                bMoving = Moving == 1;

                grid.Rows[i - 2].Cells[0].Value = PresentPosition;
                grid.Rows[i - 2].Cells[1].Value = PresentSpeed;
                grid.Rows[i - 2].Cells[2].Value = PresentLoad;
                grid.Rows[i - 2].Cells[3].Value = PresentVoltage;
                grid.Rows[i - 2].Cells[4].Value = PresentTempterature;
                grid.Rows[i - 2].Cells[5].Value = Registered;
                grid.Rows[i - 2].Cells[6].Value = Moving;

            }
        }

        private void Move(int nMove1, int nMove2)
        {
            int result;
            byte  dxl_error;


            int PresentPos1 = dynamixel.read2ByteTxRx(port_num, 1, 2, P_PRESENT_POSITION_L);
            result = dynamixel.getLastTxRxResult(port_num,1);
            if (result != COMM_SUCCESS)
            {
                System.Diagnostics.Debug.WriteLine("Pos1 failed to read " + result.ToString());
            }
            int PresentPos2 = dynamixel.read2ByteTxRx(port_num, 1, 3, P_PRESENT_POSITION_L);
            result = dynamixel.getLastTxRxResult(port_num, 1);
            if (result != COMM_SUCCESS)
            {
                System.Diagnostics.Debug.WriteLine("Pos2 failed to read " + result.ToString());
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, 1)) != 0)
            {
                System.Diagnostics.Debug.WriteLine(Marshal.PtrToStringAnsi(dynamixel.getRxPacketError(1, dxl_error)));
            }

            System.Diagnostics.Debug.WriteLine("Pos1 " + PresentPos1.ToString() + ", Pos2 " + PresentPos2.ToString());

            // Write goal position
            dynamixel.write2ByteTxRx(port_num, 1, 2, P_GOAL_POSITION_L, (ushort)(PresentPos1 + nMove1));
            result = dynamixel.getLastTxRxResult(port_num, 1);
            if (result != COMM_SUCCESS)
            {
                System.Diagnostics.Debug.WriteLine("Pos1 failed to write " + result.ToString());
            }
            dynamixel.write2ByteTxRx(port_num, 1, 3, P_GOAL_POSITION_L, (ushort)(PresentPos2 + nMove2));
            result = dynamixel.getLastTxRxResult(port_num, 1);
            if (result != COMM_SUCCESS)
            {
                System.Diagnostics.Debug.WriteLine("Pos2 failed to write " + result.ToString());
            }


        }

        private void btnBack_Click(object sender, EventArgs e)
        {
            Move(-500, 500);
        }

        private void btnFWD_Click(object sender, EventArgs e)
        {
            Move(500, -500);
        }

        private void btnCCW_Click(object sender, EventArgs e)
        {
            Move(500, 500);
        }

        private void btnCW_Click(object sender, EventArgs e)
        {
            Move(-500, -500);
        }

        private void btnAnimate_Click(object sender, EventArgs e)
        {
            if (m_animateTimer == null)
            {
                m_animateTimer = new Timer();
                m_animateTimer.Interval = 500;
                m_animateTimer.Tick += M_animateTimer_Tick;
                m_animateTimer.Start();
            }
            else
            {
                m_animateTimer.Stop();
                m_animateTimer.Dispose();
                m_animateTimer = null;
            }
        }

        private void M_animateTimer_Tick(object sender, EventArgs e)
        {
            if (bMoving)
                return;

            Move(-500, -500);
        }
    }
}
