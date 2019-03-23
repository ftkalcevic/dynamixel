namespace beveljoint
{
	partial class Form1
	{
		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		/// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
		protected override void Dispose(bool disposing)
		{
			if (disposing && (components != null))
			{
				components.Dispose();
			}
			base.Dispose(disposing);
		}

		#region Windows Form Designer generated code

		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
            this.btnCCW = new System.Windows.Forms.Button();
            this.btnCW = new System.Windows.Forms.Button();
            this.btnFWD = new System.Windows.Forms.Button();
            this.btnBack = new System.Windows.Forms.Button();
            this.grid = new System.Windows.Forms.DataGridView();
            this.colPosition = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colSpeed = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colLoad = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colVoltage = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colTemperature = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colRegistered = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.colMoving = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.btnAnimate = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.grid)).BeginInit();
            this.SuspendLayout();
            // 
            // btnCCW
            // 
            this.btnCCW.Location = new System.Drawing.Point(12, 69);
            this.btnCCW.Name = "btnCCW";
            this.btnCCW.Size = new System.Drawing.Size(71, 38);
            this.btnCCW.TabIndex = 0;
            this.btnCCW.Text = "Spin CCW";
            this.btnCCW.UseVisualStyleBackColor = true;
            this.btnCCW.Click += new System.EventHandler(this.btnCCW_Click);
            // 
            // btnCW
            // 
            this.btnCW.Location = new System.Drawing.Point(112, 69);
            this.btnCW.Name = "btnCW";
            this.btnCW.Size = new System.Drawing.Size(71, 38);
            this.btnCW.TabIndex = 1;
            this.btnCW.Text = "Spin CW";
            this.btnCW.UseVisualStyleBackColor = true;
            this.btnCW.Click += new System.EventHandler(this.btnCW_Click);
            // 
            // btnFWD
            // 
            this.btnFWD.Location = new System.Drawing.Point(62, 12);
            this.btnFWD.Name = "btnFWD";
            this.btnFWD.Size = new System.Drawing.Size(71, 38);
            this.btnFWD.TabIndex = 2;
            this.btnFWD.Text = "Move FWD";
            this.btnFWD.UseVisualStyleBackColor = true;
            this.btnFWD.Click += new System.EventHandler(this.btnFWD_Click);
            // 
            // btnBack
            // 
            this.btnBack.Location = new System.Drawing.Point(62, 124);
            this.btnBack.Name = "btnBack";
            this.btnBack.Size = new System.Drawing.Size(71, 38);
            this.btnBack.TabIndex = 3;
            this.btnBack.Text = "Move Back";
            this.btnBack.UseVisualStyleBackColor = true;
            this.btnBack.Click += new System.EventHandler(this.btnBack_Click);
            // 
            // grid
            // 
            this.grid.AllowUserToAddRows = false;
            this.grid.AllowUserToDeleteRows = false;
            this.grid.AllowUserToResizeRows = false;
            this.grid.CausesValidation = false;
            this.grid.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.grid.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.colPosition,
            this.colSpeed,
            this.colLoad,
            this.colVoltage,
            this.colTemperature,
            this.colRegistered,
            this.colMoving});
            this.grid.Location = new System.Drawing.Point(12, 207);
            this.grid.Name = "grid";
            this.grid.ReadOnly = true;
            this.grid.RowHeadersVisible = false;
            this.grid.Size = new System.Drawing.Size(707, 150);
            this.grid.TabIndex = 4;
            // 
            // colPosition
            // 
            this.colPosition.HeaderText = "Position";
            this.colPosition.Name = "colPosition";
            this.colPosition.ReadOnly = true;
            // 
            // colSpeed
            // 
            this.colSpeed.HeaderText = "Speed";
            this.colSpeed.Name = "colSpeed";
            this.colSpeed.ReadOnly = true;
            // 
            // colLoad
            // 
            this.colLoad.HeaderText = "Load";
            this.colLoad.Name = "colLoad";
            this.colLoad.ReadOnly = true;
            // 
            // colVoltage
            // 
            this.colVoltage.HeaderText = "Voltage";
            this.colVoltage.Name = "colVoltage";
            this.colVoltage.ReadOnly = true;
            // 
            // colTemperature
            // 
            this.colTemperature.HeaderText = "Temperature";
            this.colTemperature.Name = "colTemperature";
            this.colTemperature.ReadOnly = true;
            // 
            // colRegistered
            // 
            this.colRegistered.HeaderText = "Registered";
            this.colRegistered.Name = "colRegistered";
            this.colRegistered.ReadOnly = true;
            // 
            // colMoving
            // 
            this.colMoving.HeaderText = "Moving";
            this.colMoving.Name = "colMoving";
            this.colMoving.ReadOnly = true;
            // 
            // btnAnimate
            // 
            this.btnAnimate.Location = new System.Drawing.Point(315, 13);
            this.btnAnimate.Name = "btnAnimate";
            this.btnAnimate.Size = new System.Drawing.Size(75, 23);
            this.btnAnimate.TabIndex = 5;
            this.btnAnimate.Text = "Animate";
            this.btnAnimate.UseVisualStyleBackColor = true;
            this.btnAnimate.Click += new System.EventHandler(this.btnAnimate_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(816, 451);
            this.Controls.Add(this.btnAnimate);
            this.Controls.Add(this.grid);
            this.Controls.Add(this.btnBack);
            this.Controls.Add(this.btnFWD);
            this.Controls.Add(this.btnCW);
            this.Controls.Add(this.btnCCW);
            this.Name = "Form1";
            this.Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)(this.grid)).EndInit();
            this.ResumeLayout(false);

		}

        #endregion

        private System.Windows.Forms.Button btnCCW;
        private System.Windows.Forms.Button btnCW;
        private System.Windows.Forms.Button btnFWD;
        private System.Windows.Forms.Button btnBack;
        private System.Windows.Forms.DataGridView grid;
        private System.Windows.Forms.DataGridViewTextBoxColumn colPosition;
        private System.Windows.Forms.DataGridViewTextBoxColumn colSpeed;
        private System.Windows.Forms.DataGridViewTextBoxColumn colLoad;
        private System.Windows.Forms.DataGridViewTextBoxColumn colVoltage;
        private System.Windows.Forms.DataGridViewTextBoxColumn colTemperature;
        private System.Windows.Forms.DataGridViewTextBoxColumn colRegistered;
        private System.Windows.Forms.DataGridViewTextBoxColumn colMoving;
        private System.Windows.Forms.Button btnAnimate;
    }
}

