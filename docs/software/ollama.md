# LLMs on a Companion Computer

## Ollama

Ollama is a framework that simplifies running large language models (LLMs) locally on your computer, allowing for privacy and control over your data and interactions with the models.

Download [ollama](https://ollama.com/download) and see the [quickstart docs](https://github.com/ollama/ollama/blob/main/README.md#quickstart) for basic commands

Download ollama for Linux:
```
curl -fsSL https://ollama.com/install.sh | sh
```

### Key Commands

```
ollama run <model name>
ollama pull <model name>
ollama rm <model name>
ollama ls
ollama ps
ollama stop
ollama show
```

## LLM Choice

Not all models support tool calling. See the [Ollama list](https://ollama.com/search?c=tools) of the models that currently support tool calling.

## Gemma 3

See the [Gemma3](https://ollama.com/library/gemma3) models available with Ollama. Gemma 3 performance on the RPi was tested in June 2025.

### **Gemma 3 Model Performance on Raspberry Pi Compute Module 4**

**Hardware:** Raspberry Pi Compute Module 4 (8GB RAM, 32GB eMMC Storage)
**Tool:** Ollama

| Model | Prompt | Total Duration | Load Duration | Prompt Eval Count | Prompt Eval Duration | Prompt Eval Rate | Eval Count | Eval Duration | Eval Rate |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **gemma3:1b-it-qat** | "Why is the sky blue?" | 2m 25.26s | 122.09ms | 16 tokens | 1.01s | 15.82 tokens/s | 452 tokens | 2m 24.13s | 3.14 tokens/s |
| | "Write a python function to average all the values in an array." | 3m 16.13s | 116.17ms | 504 tokens | 35.89s | 14.04 tokens/s | 491 tokens | 2m 38.26s | 3.10 tokens/s |
| **gemma3:4b-it-qat** | "Why is the sky blue?" | 12m 12.76s | 117.63ms | 16 tokens | 2.03s | 7.87 tokens/s | 618 tokens | 12m 10.60s | 0.85 tokens/s |
| | "Write a python function to average all the values in an array." | Incomplete* | - | - | - | - | - | - | - |
| **gemma3:1b** | "Why is the sky blue?" | 2m 2.10s | 172.88ms | 15 tokens | 1.43s | 10.51 tokens/s | 468 tokens | 2m 0.50s | 3.88 tokens/s |
| | "Write a python function to average all the values in an array." | 2m 31.24s | 161.40ms | 505 tokens | 2.06s | 245.14 tokens/s | 567 tokens | 2m 28.91s | 3.81 tokens/s |
| **gemma3:4b** | "Why is the sky blue?" | 8m 26.39s | 166.98ms | 15 tokens | 0.74s | 20.16 tokens/s | 651 tokens | 8m 25.48s | 1.29 tokens/s |
| | "Write a python function to average all the values in an array." | 10m 29.78s | 175.75ms | 688 tokens | 8.34s | 82.51 tokens/s | 755 tokens | 10m 21.12s | 1.22 tokens/s |
| **gemma3n:e2b** | "Why is the sky blue?" | 3m 11.06s | 562.89ms | 15 tokens | 3.26s | 4.60 tokens/s | 372 tokens | 3m 7.24s | 1.99 tokens/s |
| | "Write a python function to average all the values in an array." | 4m 27.42s | 444.59ms | 22 tokens | 4.85s | 4.54 tokens/s | 515 tokens | 4m 22.11s | 1.96 tokens/s |

---
*Test was stopped prematurely due to excessive processing time and overheating of the Raspberry Pi (70°C+).*