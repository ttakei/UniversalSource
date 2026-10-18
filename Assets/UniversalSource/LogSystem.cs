using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace UniversalSource
{
    /// <summary>
    /// 4ノードのセルで、A→B→C と A→D→C の到着差を曲率のログとして残す。
    /// StabilizeStep は関連リンクの補正行列を、その差が縮む向きへ微小に戻す。
    /// 横軸はログ自身の歩数であり、ワールド時刻でも絶対座標でもない。
    /// </summary>
    public readonly struct CurvatureResidual
    {
        public CurvatureResidual(
            int logStep,
            int sourceProperTime,
            int arrivalProperTime,
            Vector4 residual,
            float energy,
            float plaquetteEnergy,
            Vector4 alongAbc,
            Vector4 alongAdc)
        {
            LogStep = logStep;
            SourceProperTime = sourceProperTime;
            ArrivalProperTime = arrivalProperTime;
            Residual = residual;
            Energy = energy;
            PlaquetteEnergy = plaquetteEnergy;
            AlongAbc = alongAbc;
            AlongAdc = alongAdc;
        }

        public int LogStep { get; }
        public int SourceProperTime { get; }
        public int ArrivalProperTime { get; }
        public Vector4 Residual { get; }
        public float Energy { get; }
        public float PlaquetteEnergy { get; }
        public Vector4 AlongAbc { get; }
        public Vector4 AlongAdc { get; }
    }

    [DisallowMultipleComponent]
    [DefaultExecutionOrder(100)]
    [AddComponentMenu("UniversalSource/Log System")]
    public sealed class LogSystem : MonoBehaviour
    {
        public const int LogCapacity = 96;
        const float SingularDeterminant = 1e-6f;
        const float EnergyRiseTolerance = 1e-5f;

        [SerializeField] SourceNode nodeA = null;
        [SerializeField] SourceNode nodeB = null;
        [SerializeField] SourceNode nodeC = null;
        [SerializeField] SourceNode nodeD = null;
        [SerializeField] UniversalLink linkAB = null;
        [SerializeField] UniversalLink linkBC = null;
        [SerializeField] UniversalLink linkAD = null;
        [SerializeField] UniversalLink linkDC = null;
        [SerializeField] float feedbackGain = 0.1f;
        [SerializeField] int demoStepsOnStart = 24;
        [SerializeField] bool showOverlay = true;

        readonly List<CurvatureResidual> log = new List<CurvatureResidual>(LogCapacity);
        int logStep;
        int feedbackStep;
        float initialPlaquetteEnergy = -1f;
        float accumulatedLogEnergy;
        string overlayText = "";
        string status = "";
        Vector2 scroll;
        GUIStyle bodyStyle;

        public IReadOnlyList<CurvatureResidual> Residuals => log;
        public float InitialPlaquetteEnergy => initialPlaquetteEnergy;
        public float AccumulatedLogEnergy => accumulatedLogEnergy;
        public string Status => status;

        public float CurrentPlaquetteEnergy
        {
            get { return log.Count == 0 ? 0f : log[log.Count - 1].PlaquetteEnergy; }
        }

        public CurvatureResidual Latest
        {
            get { return log.Count == 0 ? default : log[log.Count - 1]; }
        }

        void Start()
        {
            if (!RecordCurrent())
            {
                Debug.LogWarning(status);
                return;
            }

            RunSteps(demoStepsOnStart);
            Debug.Log(overlayText);
        }

        void OnGUI()
        {
            if (!showOverlay)
                return;

            if (bodyStyle == null)
            {
                bodyStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 14,
                    wordWrap = true,
                    richText = false
                };
                bodyStyle.normal.textColor = new Color(0.1f, 0.12f, 0.14f);
            }

            var width = Mathf.Min(540f, Screen.width - 32f);
            var height = Mathf.Min(470f, Screen.height - 32f);
            float x;
            float y;
            if (Screen.width >= 860f + width + 48f)
            {
                x = Screen.width - width - 16f;
                y = 16f;
            }
            else
            {
                x = 16f;
                y = Mathf.Max(16f, Screen.height - height - 16f);
            }

            GUILayout.BeginArea(new Rect(x, y, width, height), GUI.skin.box);
            if (GUILayout.Button("安定化を8ステップ進める"))
            {
                RunSteps(8);
                Debug.Log(overlayText);
            }

            var chart = GUILayoutUtility.GetRect(10f, 150f, GUILayout.ExpandWidth(true));
            DrawEnergyChart(chart);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label(string.IsNullOrEmpty(overlayText) ? "ログ待ち" : overlayText, bodyStyle);
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        /// <summary>セルの残差を測る。ログへは足さない。</summary>
        public bool TryMeasureCurvature(out CurvatureResidual residual)
        {
            residual = default;
            if (!HasCell() || !TryReadPathMatrices(out var ab, out var bc, out var ad, out var dc))
            {
                status = "セルのリンクを読めない。";
                return false;
            }

            var pathAbc = bc * ab;
            var pathAdc = dc * ad;
            var probe = CommunicationProbe();
            if (!UniversalLink.TryTransportAlong(nodeA, probe, out var alongAbc, linkAB, linkBC)
                || !UniversalLink.TryTransportAlong(nodeA, probe, out var alongAdc, linkAD, linkDC))
            {
                status = "経路の転送を完了できない。";
                return false;
            }

            var residualVector = alongAbc - alongAdc;
            residual = new CurvatureResidual(
                logStep,
                nodeA.ProperTime,
                nodeC.ProperTime,
                residualVector,
                residualVector.sqrMagnitude,
                FrobeniusGap(pathAbc, pathAdc),
                alongAbc,
                alongAdc);
            return true;
        }

        /// <summary>いまの残差をログへ1件足す。</summary>
        public bool RecordCurrent()
        {
            if (!TryMeasureCurvature(out var residual))
                return false;

            Append(residual);
            logStep++;
            RefreshOverlay();
            return true;
        }

        /// <summary>
        /// 経路差が縮む向きに、関連する4リンクの補正行列を1回だけ更新し、結果をログへ足す。
        /// 偶数歩は到着側（B→C と D→C）、奇数歩は出発側（A→B と A→D）を動かす。
        /// </summary>
        [ContextMenu("安定化を1ステップ")]
        public bool StabilizeStep()
        {
            if (!HasCell())
            {
                status = "セルの参照を確認する。";
                RefreshOverlay();
                return false;
            }

            if (log.Count == 0 && !RecordCurrent())
                return false;

            if (!TryReadPathMatrices(out var ab, out var bc, out var ad, out var dc))
                return false;

            var before = FrobeniusGap(bc * ab, dc * ad);
            var stored = CaptureStored();
            var gain = Mathf.Clamp(feedbackGain, 0f, 0.5f);
            var adjustArrivalLinks = (feedbackStep % 2) == 0;
            if (!TryApplyFeedback(ab, bc, ad, dc, gain, adjustArrivalLinks))
            {
                Restore(stored);
                status = "補正行列を更新できない。";
                RefreshOverlay();
                return false;
            }

            feedbackStep++;
            if (!TryReadPathMatrices(out ab, out bc, out ad, out dc))
            {
                Restore(stored);
                status = "更新後のリンクを読めない。";
                RefreshOverlay();
                return false;
            }

            var after = FrobeniusGap(bc * ab, dc * ad);
            if (after > before + EnergyRiseTolerance)
            {
                Restore(stored);
                status = "この歩はエネルギーが増えるため、行列を戻した。";
                RecordCurrent();
                return true;
            }

            status = string.Format(
                CultureInfo.InvariantCulture,
                "歩 {0}: エネルギー {1:F5} → {2:F5}",
                feedbackStep,
                before,
                after);
            RecordCurrent();
            return true;
        }

        [ContextMenu("安定化を8ステップ")]
        public int RunSteps(int count)
        {
            var ran = 0;
            var steps = Mathf.Max(0, count);
            for (var i = 0; i < steps; i++)
            {
                if (!StabilizeStep())
                    break;
                ran++;
            }

            RefreshOverlay();
            return ran;
        }

        public void ClearLog()
        {
            log.Clear();
            logStep = 0;
            feedbackStep = 0;
            initialPlaquetteEnergy = -1f;
            accumulatedLogEnergy = 0f;
            status = "";
            overlayText = "";
        }

        bool TryApplyFeedback(Matrix4x4 ab, Matrix4x4 bc, Matrix4x4 ad, Matrix4x4 dc, float gain, bool adjustArrivalLinks)
        {
            var pathAbc = bc * ab;
            var pathAdc = dc * ad;
            var target = Lerp(pathAbc, pathAdc, 0.5f);

            if (adjustArrivalLinks)
            {
                if (!TryInvert(ab, out var inverseAb) || !TryInvert(ad, out var inverseAd))
                    return false;
                var nextBc = NormalizeLerp(bc, target * inverseAb, gain);
                var nextDc = NormalizeLerp(dc, target * inverseAd, gain);
                return nextBc.HasValue
                    && nextDc.HasValue
                    && TrySetPathMatrix(linkBC, nodeB, nextBc.Value)
                    && TrySetPathMatrix(linkDC, nodeD, nextDc.Value);
            }

            if (!TryInvert(bc, out var inverseBc) || !TryInvert(dc, out var inverseDc))
                return false;

            var nextAb = NormalizeLerp(ab, inverseBc * target, gain);
            var nextAd = NormalizeLerp(ad, inverseDc * target, gain);
            return nextAb.HasValue
                && nextAd.HasValue
                && TrySetPathMatrix(linkAB, nodeA, nextAb.Value)
                && TrySetPathMatrix(linkAD, nodeA, nextAd.Value);
        }

        bool HasCell()
        {
            return nodeA != null
                && nodeB != null
                && nodeC != null
                && nodeD != null
                && linkAB != null
                && linkBC != null
                && linkAD != null
                && linkDC != null
                && linkAB.Connects(nodeA) && linkAB.Connects(nodeB)
                && linkBC.Connects(nodeB) && linkBC.Connects(nodeC)
                && linkAD.Connects(nodeA) && linkAD.Connects(nodeD)
                && linkDC.Connects(nodeD) && linkDC.Connects(nodeC);
        }

        bool TryReadPathMatrices(out Matrix4x4 ab, out Matrix4x4 bc, out Matrix4x4 ad, out Matrix4x4 dc)
        {
            ab = Matrix4x4.identity;
            bc = Matrix4x4.identity;
            ad = Matrix4x4.identity;
            dc = Matrix4x4.identity;
            return linkAB.TryGetTransportMatrix(nodeA, out ab)
                && linkBC.TryGetTransportMatrix(nodeB, out bc)
                && linkAD.TryGetTransportMatrix(nodeA, out ad)
                && linkDC.TryGetTransportMatrix(nodeD, out dc);
        }

        bool TrySetPathMatrix(UniversalLink link, SourceNode from, Matrix4x4 pathMatrix)
        {
            if (link.EndpointA == from)
            {
                link.SetTransportMatrix(pathMatrix);
                return true;
            }

            if (link.EndpointB == from && TryInvert(pathMatrix, out var stored))
            {
                link.SetTransportMatrix(stored);
                return true;
            }

            return false;
        }

        Vector4 CommunicationProbe()
        {
            var state = nodeA.State;
            if (state.sqrMagnitude < 1e-8f)
                return new Vector4(1f, 0f, 0f, 0f);
            return state;
        }

        StoredMatrices CaptureStored()
        {
            return new StoredMatrices(
                linkAB.TransportMatrix,
                linkBC.TransportMatrix,
                linkAD.TransportMatrix,
                linkDC.TransportMatrix);
        }

        void Restore(StoredMatrices stored)
        {
            linkAB.SetTransportMatrix(stored.Ab);
            linkBC.SetTransportMatrix(stored.Bc);
            linkAD.SetTransportMatrix(stored.Ad);
            linkDC.SetTransportMatrix(stored.Dc);
        }

        void Append(CurvatureResidual residual)
        {
            if (initialPlaquetteEnergy < 0f)
                initialPlaquetteEnergy = residual.PlaquetteEnergy;

            accumulatedLogEnergy += residual.PlaquetteEnergy;
            if (log.Count == LogCapacity)
                log.RemoveAt(0);
            log.Add(residual);
        }

        void RefreshOverlay()
        {
            var culture = CultureInfo.InvariantCulture;
            var latest = Latest;
            var lines = new List<string>
            {
                log.Count == 0
                    ? "結果: ログは空。"
                    : string.Format(
                        culture,
                        "セルエネルギー: 開始 {0:F5}、現在 {1:F5}。蓄積ログの合計 {2:F5}。",
                        initialPlaquetteEnergy,
                        CurrentPlaquetteEnergy,
                        accumulatedLogEnergy),
                "横軸はログの歩数。棒は各歩のセルエネルギー。",
                status
            };

            if (log.Count > 0)
            {
                lines.Add(string.Format(
                    culture,
                    "残差 A→B→C − A→D→C: {0}。大きさの二乗 {1:F5}。",
                    FormatVector(latest.Residual),
                    latest.Energy));
                lines.Add(string.Format(
                    culture,
                    "A→B→C {0}",
                    FormatVector(latest.AlongAbc)));
                lines.Add(string.Format(
                    culture,
                    "A→D→C {0}",
                    FormatVector(latest.AlongAdc)));
                lines.Add(string.Format(
                    culture,
                    "ログ歩 {0}。Aの固有時間 {1}、Cの固有時間 {2}。",
                    latest.LogStep,
                    latest.SourceProperTime,
                    latest.ArrivalProperTime));
            }

            overlayText = string.Join("\n", lines);
        }

        void DrawEnergyChart(Rect rect)
        {
            GUI.Box(rect, GUIContent.none);
            if (log.Count == 0)
                return;

            var maxEnergy = 0f;
            for (var i = 0; i < log.Count; i++)
                maxEnergy = Mathf.Max(maxEnergy, log[i].PlaquetteEnergy);
            if (maxEnergy < 1e-8f)
                maxEnergy = 1e-8f;

            var inner = new Rect(rect.x + 6f, rect.y + 6f, rect.width - 12f, rect.height - 12f);
            var slot = inner.width / log.Count;
            var previous = GUI.color;
            for (var i = 0; i < log.Count; i++)
            {
                var height = inner.height * (log[i].PlaquetteEnergy / maxEnergy);
                var bar = new Rect(
                    inner.x + slot * i + 1f,
                    inner.y + inner.height - height,
                    Mathf.Max(1f, slot - 2f),
                    Mathf.Max(1f, height));
                var age = log.Count == 1 ? 1f : i / (float)(log.Count - 1);
                GUI.color = Color.Lerp(new Color(0.75f, 0.35f, 0.2f), new Color(0.15f, 0.4f, 0.55f), age);
                GUI.DrawTexture(bar, Texture2D.whiteTexture);
            }

            GUI.color = previous;
        }

        static Matrix4x4? NormalizeLerp(Matrix4x4 current, Matrix4x4 target, float gain)
        {
            var blended = Lerp(current, target, gain);
            if (!TryNormalizeDeterminant(ref blended))
                return null;
            return blended;
        }

        static Matrix4x4 Lerp(Matrix4x4 from, Matrix4x4 to, float gain)
        {
            var result = from;
            var keep = 1f - gain;
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                    result[row, column] = keep * from[row, column] + gain * to[row, column];
            }

            return result;
        }

        static Matrix4x4 MultiplyScalar(Matrix4x4 matrix, float scalar)
        {
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                    matrix[row, column] *= scalar;
            }

            return matrix;
        }

        static bool TryNormalizeDeterminant(ref Matrix4x4 matrix)
        {
            var determinant = matrix.determinant;
            var absolute = Mathf.Abs(determinant);
            if (absolute < SingularDeterminant || float.IsNaN(determinant) || float.IsInfinity(determinant))
                return false;

            var scale = Mathf.Pow(absolute, -0.25f);
            matrix = MultiplyScalar(matrix, scale);
            return true;
        }

        static bool TryInvert(Matrix4x4 matrix, out Matrix4x4 inverse)
        {
            if (Mathf.Abs(matrix.determinant) < SingularDeterminant)
            {
                inverse = Matrix4x4.identity;
                return false;
            }

            inverse = matrix.inverse;
            return true;
        }

        static float FrobeniusGap(Matrix4x4 left, Matrix4x4 right)
        {
            var sum = 0f;
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                {
                    var delta = left[row, column] - right[row, column];
                    sum += delta * delta;
                }
            }

            return sum;
        }

        static string FormatVector(Vector4 value)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "({0:F4}, {1:F4}, {2:F4}, {3:F4})",
                value.x,
                value.y,
                value.z,
                value.w);
        }

        readonly struct StoredMatrices
        {
            public StoredMatrices(Matrix4x4 ab, Matrix4x4 bc, Matrix4x4 ad, Matrix4x4 dc)
            {
                Ab = ab;
                Bc = bc;
                Ad = ad;
                Dc = dc;
            }

            public Matrix4x4 Ab { get; }
            public Matrix4x4 Bc { get; }
            public Matrix4x4 Ad { get; }
            public Matrix4x4 Dc { get; }
        }
    }
}
