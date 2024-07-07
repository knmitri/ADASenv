from openai import OpenAI

client = OpenAI(api_key="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")

response = client.images.generate(
  model="dall-e-3",
  prompt="a white siamese cat",
  size="1024x1024",
  quality="standard",
  n=1,
)

image_url = response.data[0].url
